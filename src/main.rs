#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::entry;
use nb::block;
use stm32f1xx_hal::{
    gpio::PinState,
    pac::{self, interrupt},
    prelude::*,
    rtc::Rtc,
    serial::{Config, Serial},
    timer::Timer,
};

#[panic_handler]
fn fn_on_panic(_info: &PanicInfo) -> ! {
    loop {}
}

struct Application {
    timer: stm32f1xx_hal::timer::SysCounterHz,
    tx: stm32f1xx_hal::serial::Tx<stm32f1xx_hal::pac::USART1>,
    led_red: stm32f1xx_hal::gpio::Pin<
        stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
        stm32f1xx_hal::gpio::CRL,
        'B',
        5,
    >,
    led_green: stm32f1xx_hal::gpio::Pin<
        stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
        stm32f1xx_hal::gpio::CRL,
        'B',
        0,
    >,
    rtc: Rtc,
    current_led: bool,
}

static mut APPLICATION: Option<Application> = None;

impl Application {
    fn init() {
        let cp = cortex_m::Peripherals::take().unwrap();
        let dp = pac::Peripherals::take().unwrap();

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8_000_000.Hz())
            .sysclk(72_000_000.Hz())
            .hclk(72_000_000.Hz())
            .adcclk(12_000_000.Hz())
            .pclk1(36_000_000.Hz())
            .pclk2(72_000_000.Hz())
            .freeze(&mut flash.acr);

        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();
        let mut afio = dp.AFIO.constrain();

        let mut pwr = dp.PWR;
        let mut backup_domain = rcc.bkp.constrain(dp.BKP, &mut pwr);
        let mut rtc = Rtc::new(dp.RTC, &mut backup_domain);
        rtc.select_frequency(1.Hz());
        rtc.listen_seconds();

        let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx = gpioa.pa10;
        let serial = Serial::usart1(
            dp.USART1,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(115_200.bps()),
            clocks,
        );
        let (tx, _rx) = serial.split();

        let led_red = gpiob
            .pb5
            .into_push_pull_output_with_state(&mut gpiob.crl, PinState::High);

        let led_green = gpiob
            .pb0
            .into_push_pull_output_with_state(&mut gpiob.crl, PinState::High);

        let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
        timer.start(8.Hz()).unwrap();

        let application = Application {
            timer,
            tx,
            led_red,
            led_green,
            rtc,
            current_led: false,
        };

        unsafe {
            APPLICATION = Some(application);
            cortex_m::peripheral::NVIC::unmask(stm32f1xx_hal::pac::Interrupt::RTC);
        }
    }

    fn run_internal(&mut self) -> ! {
        loop {
            // cortex_m::asm::wfi();
            block!(self.timer.wait()).unwrap();
            self.led_red.set_low();
            block!(self.timer.wait()).unwrap();
            self.led_red.set_high();
        }
    }

    fn on_rtc_internal(&mut self) {
        self.rtc.clear_second_flag();

        if self.current_led {
            self.current_led = false;
            self.led_green.set_high();
        } else {
            self.current_led = true;
            self.led_green.set_low();
        }

        block!(self.tx.write(b'A')).unwrap();
    }

    pub unsafe fn run() -> ! {
        Self::init();
        Self::get_application().run_internal();
    }

    pub unsafe fn on_rtc() {
        Self::get_application().on_rtc_internal();
    }

    unsafe fn get_application() -> &'static mut Application {
        if let Some(application) = APPLICATION.as_mut() {
            return application;
        } else {
            panic!("No application");
        }
    }
}

#[entry]
unsafe fn main() -> ! {
    Application::run();
}

#[interrupt]
unsafe fn RTC() {
    Application::on_rtc()
}
