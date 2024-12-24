#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal;
use cortex_m::prelude::*;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::*;
use fugit::ExtU32;
use hal::pac;
use panic_probe as _;
use rotary_encoder_embedded::Direction;
use rotary_encoder_embedded::RotaryEncoder;
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_human_interface_device::page::Keyboard;
use usbd_human_interface_device::prelude::*;

use rp_pico as bsp;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sio = bsp::hal::sio::Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Starting");

    //USB
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut keyboard = UsbHidClassBuilder::new()
        .add_device(usbd_human_interface_device::device::keyboard::BootKeyboardConfig::default())
        .build(&usb_bus);

    //https://pid.codes
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .strings(&[StringDescriptors::default()
            .manufacturer("usbd-human-interface-device")
            .product("Boot Keyboard")
            .serial_number("TEST_2")])
        .unwrap()
        .build();

    //GPIO pins
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_low().unwrap();

    // Configure DT and CLK pins, typically pullup input
    let lr_rotary_dt = pins.gpio0.into_pull_up_input();
    let lr_rotary_clk = pins.gpio1.into_pull_up_input();
    let btn_enter = pins.gpio2.into_pull_up_input();

    // Initialize the rotary encoder
    let mut lr_rotary_encoder =
        RotaryEncoder::new(lr_rotary_dt, lr_rotary_clk).into_standard_mode();

    let ud_rotary_dt = pins.gpio10.into_pull_up_input();
    let ud_rotary_clk = pins.gpio11.into_pull_up_input();
    let btn_esc = pins.gpio12.into_pull_up_input();

    // Initialize the rotary encoder
    let mut ud_rotary_encoder =
        RotaryEncoder::new(ud_rotary_dt, ud_rotary_clk).into_standard_mode();

    let mut input_count_down = timer.count_down();
    input_count_down.start(2.millis());

    let mut tick_count_down = timer.count_down();
    tick_count_down.start(1.millis());

    loop {
        //Poll the keys every 2ms
        if input_count_down.wait().is_ok() {
            let lr_dir = lr_rotary_encoder.update();
            let enter = btn_enter.as_input().is_low().unwrap();

            let ud_dir = ud_rotary_encoder.update();
            let esc = btn_esc.as_input().is_low().unwrap();

            let keys = get_keys(lr_dir, enter, ud_dir, esc);
            match keyboard.device().write_report(keys) {
                Err(UsbHidError::WouldBlock) => {}
                Err(UsbHidError::Duplicate) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to write keyboard report: {:?}", e)
                }
            };
        }

        //Tick once per ms
        if tick_count_down.wait().is_ok() {
            match keyboard.tick() {
                Err(UsbHidError::WouldBlock) => {}
                Ok(_) => {}
                Err(e) => {
                    core::panic!("Failed to process keyboard tick: {:?}", e)
                }
            };
        }
        if usb_dev.poll(&mut [&mut keyboard]) {
            match keyboard.device().read_report() {
                Err(UsbError::WouldBlock) => {
                    //do nothing
                }
                Err(e) => {
                    core::panic!("Failed to read keyboard report: {:?}", e)
                }
                Ok(_leds) => {
                    //led_pin.set_state(PinState::from(true)).ok();
                }
            }
        }
    }
}

fn get_keys(lr_dir: Direction, enter: bool, ud_dir: Direction, esc: bool) -> [Keyboard; 6] {
    [
        if lr_dir == Direction::Anticlockwise {
            Keyboard::LeftArrow
        } else {
            Keyboard::NoEventIndicated
        }, //Left
        if lr_dir == Direction::Clockwise {
            Keyboard::RightArrow
        } else {
            Keyboard::NoEventIndicated
        },
        if enter {
            Keyboard::ReturnEnter
        } else {
            Keyboard::NoEventIndicated
        },
        if ud_dir == Direction::Anticlockwise {
            Keyboard::DownArrow
        } else {
            Keyboard::NoEventIndicated
        }, //Left
        if ud_dir == Direction::Clockwise {
            Keyboard::UpArrow
        } else {
            Keyboard::NoEventIndicated
        },
        if esc {
            Keyboard::Escape
        } else {
            Keyboard::NoEventIndicated
        },
    ]
}
