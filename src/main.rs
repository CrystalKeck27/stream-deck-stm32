#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    ltdc::{
        self, Ltdc, LtdcConfiguration, LtdcLayer, LtdcLayerConfig, PolarityActive, PolarityEdge,
    },
    pac::LTDC as LTDC2,
    peripherals, Config,
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const DISPLAY_WIDTH: usize = 480;
const DISPLAY_HEIGHT: usize = 272;

pub static mut FB1: [TargetPixelType; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
    [0; DISPLAY_WIDTH * DISPLAY_HEIGHT];
pub static mut FB2: [TargetPixelType; DISPLAY_WIDTH * DISPLAY_HEIGHT] =
    [0; DISPLAY_WIDTH * DISPLAY_HEIGHT];

bind_interrupts!(struct Irqs {
    LTDC => ltdc::InterruptHandler<peripherals::LTDC>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = rcc_setup::stm32f746g_init();
    info!("Hello World!");

    // let mut led = Output::new(p.PI1, Level::High, Speed::Low);
    let mut button = ExtiInput::new(p.PI11, p.EXTI11, Pull::Up);

    const RK043FN48H_HSYNC: u16 = 41; // Horizontal synchronization
    const RK043FN48H_HBP: u16 = 13; // Horizontal back porch
    const RK043FN48H_HFP: u16 = 32; // Horizontal front porch
    const RK043FN48H_VSYNC: u16 = 10; // Vertical synchronization
    const RK043FN48H_VBP: u16 = 2; // Vertical back porch
    const RK043FN48H_VFP: u16 = 2; // Vertical front porch

    // NOTE: all polarities have to be reversed with respect to the STM32U5G9J-DK2 CubeMX parametrization
    let ltdc_config = LtdcConfiguration {
        active_width: DISPLAY_WIDTH as _,
        active_height: DISPLAY_HEIGHT as _,
        h_back_porch: RK043FN48H_HBP,
        h_front_porch: RK043FN48H_HFP,
        v_back_porch: RK043FN48H_VBP,
        v_front_porch: RK043FN48H_VFP,
        h_sync: RK043FN48H_HSYNC,
        v_sync: RK043FN48H_VSYNC,
        h_sync_polarity: PolarityActive::ActiveLow,
        v_sync_polarity: PolarityActive::ActiveLow,
        data_enable_polarity: PolarityActive::ActiveLow,
        pixel_clock_polarity: PolarityEdge::RisingEdge,
    };

    info!("Creating LTDC");
    let mut ltdc_de = Output::new(p.PK7, Level::Low, Speed::High);
    let mut ltdc = Ltdc::new_with_pins(
        p.LTDC, Irqs, p.PI14, p.PI10, p.PI9, p.PE4, p.PJ13, p.PD6, p.PG11, p.PI4, p.PI5, p.PB8,
        p.PB9, p.PE5, p.PE6, p.PH13, p.PG10, p.PH15, p.PI0, p.PI1, p.PD3, p.PG13, p.PH3, p.PA1,
        p.PH9, p.PA11, p.PA12, p.PA8, p.PG6,
    );
    info!("Initializing LTDC");
    ltdc.init(&ltdc_config);
    ltdc_de.set_low();

    info!("enable bottom layer");
    let layer_config = LtdcLayerConfig {
        pixel_format: ltdc::PixelFormat::RGB565, // 2 bytes per pixel
        layer: LtdcLayer::Layer1,
        window_x0: 0,
        window_x1: DISPLAY_WIDTH as _,
        window_y0: 0,
        window_y1: DISPLAY_HEIGHT as _,
    };

    ltdc.init_layer(&layer_config, None);

    ltdc.enable();

    info!("filling framebuffer");
    for i in 0..DISPLAY_WIDTH * DISPLAY_HEIGHT {
        unsafe {
            FB1[i] = 0x00FF;
        }
    }

    info!("setting buffer");
    unsafe {
        unwrap!(ltdc.set_buffer(LtdcLayer::Layer1, FB1.as_ptr() as *const _).await);
    }

    info!("done");
    loop {
        button.wait_for_any_edge().await;
        info!("high");
        // led.set_high();
        Timer::after_millis(300).await;

        button.wait_for_any_edge().await;
        info!("low");
        // led.set_low();
        Timer::after_millis(300).await;
    }
}

type TargetPixelType = u16;

mod rcc_setup {

    use embassy_stm32::rcc::*;
    use embassy_stm32::{Config, Peripherals};

    pub fn stm32f746g_init() -> Peripherals {
        let mut config = Config::default();
        config.rcc.hse = None;
        config.rcc.hsi = true;
        config.rcc.pll_src = PllSource::HSI;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL50,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV2),
            divr: None,
        });
        config.rcc.pllsai = Some(Pll {
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL96,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV2),
            divr: Some(PllRDiv::DIV5),
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;

        embassy_stm32::init(config)
    }
}
