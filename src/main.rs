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
use rotary_encoder_embedded::standard::StandardMode;
use rotary_encoder_embedded::Direction;
use rotary_encoder_embedded::RotaryEncoder;
use rp2040_hal::gpio::FunctionSio;
use rp2040_hal::gpio::Pin;
use rp2040_hal::gpio::PinId;
use rp2040_hal::gpio::PullUp;
use rp2040_hal::gpio::SioInput;
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

    let mut top_knob = Knob::new(
        pins.gpio11.into_pull_up_input(),
        pins.gpio12.into_pull_up_input(),
        pins.gpio13.into_pull_up_input(),
        Keyboard::LeftArrow,
        Keyboard::RightArrow,
        Keyboard::ReturnEnter,
    );

    let mut lower_knob = Knob::new(
        pins.gpio21.into_pull_up_input(),
        pins.gpio20.into_pull_up_input(),
        pins.gpio19.into_pull_up_input(),
        Keyboard::DownArrow,
        Keyboard::UpArrow,
        Keyboard::Escape,
    );

    let mut bright_knob = Knob::new(
        pins.gpio27.into_pull_up_input(),
        pins.gpio26.into_pull_up_input(),
        pins.gpio22.into_pull_up_input(),
        Keyboard::X,
        Keyboard::Y,
        Keyboard::Z,
    );

    let mut volume_knob = Knob::new(
        pins.gpio18.into_pull_up_input(),
        pins.gpio17.into_pull_up_input(),
        pins.gpio16.into_pull_up_input(),
        Keyboard::A,
        Keyboard::B,
        Keyboard::F5,
    );

    let mut input_count_down = timer.count_down();
    input_count_down.start(2.millis());

    let mut tick_count_down = timer.count_down();
    tick_count_down.start(1.millis());

    loop {
        //Poll the keys every 2ms
        if input_count_down.wait().is_ok() {
            let keys = [
                top_knob.update(),
                lower_knob.update(),
                bright_knob.update(),
                volume_knob.update(),
            ];
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

// push, left, right.
// 4 knobs.
struct Knob<DT: PinId, CLK: PinId, BTN: PinId> {
    encoder: RotaryEncoder<
        StandardMode,
        Pin<DT, FunctionSio<SioInput>, PullUp>,
        Pin<CLK, FunctionSio<SioInput>, PullUp>,
    >,
    button: Pin<BTN, FunctionSio<SioInput>, PullUp>,
    on_counterclockwise: Keyboard,
    on_clockwise: Keyboard,
    on_press: Keyboard,
}

impl<DT, CLK, BTN> Knob<DT, CLK, BTN>
where
    DT: PinId,
    CLK: PinId,
    BTN: PinId,
{
    fn new(
        dt: Pin<DT, FunctionSio<SioInput>, PullUp>,
        clk: Pin<CLK, FunctionSio<SioInput>, PullUp>,
        btn: Pin<BTN, FunctionSio<SioInput>, PullUp>,
        on_counterclockwise: Keyboard,
        on_clockwise: Keyboard,
        on_press: Keyboard,
    ) -> Self {
        let encoder = RotaryEncoder::new(dt, clk).into_standard_mode();
        Self {
            encoder,
            button: btn,
            on_counterclockwise,
            on_clockwise,
            on_press,
        }
    }

    fn update(&mut self) -> Keyboard {
        let direction = self.encoder.update();
        let btn = self.button.as_input().is_low().unwrap();
        println!("{}", btn);
        match direction {
            Direction::Anticlockwise => self.on_counterclockwise,
            Direction::Clockwise => self.on_clockwise,
            Direction::None if btn => self.on_press,
            _ => Keyboard::NoEventIndicated,
        }
    }
}
