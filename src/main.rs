#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    exti::ExtiInput,
    fmc::Fmc,
    gpio::{Level, Output, Pull, Speed},
    i2c::{self, I2c},
    ltdc::{
        self, Ltdc, LtdcConfiguration, LtdcLayer, LtdcLayerConfig, PolarityActive, PolarityEdge,
    },
    peripherals,
    time::Hertz,
};
use embassy_time::{Delay, Timer};
use {defmt_rtt as _, panic_probe as _};

const DISPLAY_WIDTH: usize = 480;
const DISPLAY_HEIGHT: usize = 272;

bind_interrupts!(struct Irqs {
    LTDC => ltdc::InterruptHandler<peripherals::LTDC>;

    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = rcc_setup::stm32f746g_init();
    info!("Hello World!");

    let mut led = Output::new(p.PI1, Level::High, Speed::Low);
    let mut button = ExtiInput::new(p.PI11, p.EXTI11, Pull::Up);
    let mut touchscreen = ExtiInput::new(p.PI13, p.EXTI13, Pull::Down);

    let mut core_peri = cortex_m::Peripherals::take().unwrap();

    core_peri.SCB.enable_icache();

    core_peri.DCB.enable_trace();
    core_peri.DWT.enable_cycle_counter();

    let sdram_size = 16 * 1024 * 1024; // 16MB

    {
        let mpu = core_peri.MPU;
        let scb = &mut core_peri.SCB;
        let size = sdram_size;
        // Refer to ARM®v7-M Architecture Reference Manual ARM DDI 0403
        // Version E.b Section B3.5
        const MEMFAULTENA: u32 = 1 << 16;

        unsafe {
            /* Make sure outstanding transfers are done */
            cortex_m::asm::dmb();

            scb.shcsr.modify(|r| r & !MEMFAULTENA);

            /* Disable the MPU and clear the control register*/
            mpu.ctrl.write(0);
        }

        const REGION_NUMBER0: u32 = 0x00;
        const REGION_BASE_ADDRESS: u32 = 0xC000_0000;

        const REGION_FULL_ACCESS: u32 = 0x03;
        const REGION_CACHEABLE: u32 = 0x01;
        const REGION_WRITE_BACK: u32 = 0x01;
        const REGION_ENABLE: u32 = 0x01;

        crate::assert_eq!(
            size & (size - 1),
            0,
            "SDRAM memory region size must be a power of 2"
        );
        crate::assert_eq!(
            size & 0x1F,
            0,
            "SDRAM memory region size must be 32 bytes or more"
        );
        fn log2minus1(sz: u32) -> u32 {
            for i in 5..=31 {
                if sz == (1 << i) {
                    return i - 1;
                }
            }
            crate::panic!("Unknown SDRAM memory region size!");
        }

        //info!("SDRAM Memory Size 0x{:x}", log2minus1(size as u32));

        // Configure region 0
        //
        // Cacheable, outer and inner write-back, no write allocate. So
        // reads are cached, but writes always write all the way to SDRAM
        unsafe {
            mpu.rnr.write(REGION_NUMBER0);
            mpu.rbar.write(REGION_BASE_ADDRESS);
            mpu.rasr.write(
                (REGION_FULL_ACCESS << 24)
                    | (REGION_CACHEABLE << 17)
                    | (REGION_WRITE_BACK << 16)
                    | (log2minus1(size as u32) << 1)
                    | REGION_ENABLE,
            );
        }

        const MPU_ENABLE: u32 = 0x01;
        const MPU_DEFAULT_MMAP_FOR_PRIVILEGED: u32 = 0x04;

        // Enable
        unsafe {
            mpu.ctrl
                .modify(|r| r | MPU_DEFAULT_MMAP_FOR_PRIVILEGED | MPU_ENABLE);

            scb.shcsr.modify(|r| r | MEMFAULTENA);

            // Ensure MPU settings take effect
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }
    }

    let mut sdram = Fmc::sdram_a12bits_d16bits_4banks_bank1(
        p.FMC,
        // a0-a11
        p.PF0,  // FMC_A0
        p.PF1,  // FMC_A1
        p.PF2,  // FMC_A2
        p.PF3,  // FMC_A3
        p.PF4,  // FMC_A4
        p.PF5,  // FMC_A5
        p.PF12, // FMC_A6
        p.PF13, // FMC_A7
        p.PF14, // FMC_A8
        p.PF15, // FMC_A9
        p.PG0,  // FMC_A10
        p.PG1,  // FMC_A11
        // BA0, BA1
        p.PG4, // FMC_BA0
        p.PG5, // FMC_BA1
        // D0-D15
        p.PD14, // FMC_D0
        p.PD15, // FMC_D1
        p.PD0,  // FMC_D2
        p.PD1,  // FMC_D3
        p.PE7,  // FMC_D4
        p.PE8,  // FMC_D5
        p.PE9,  // FMC_D6
        p.PE10, // FMC_D7
        p.PE11, // FMC_D8
        p.PE12, // FMC_D9
        p.PE13, // FMC_D10
        p.PE14, // FMC_D11
        p.PE15, // FMC_D12
        p.PD8,  // FMC_D13
        p.PD9,  // FMC_D14
        p.PD10, // FMC_D15
        // NBL0, NBL1
        p.PE0, // FMC_NBL0
        p.PE1, // FMC_NBL1
        // SDCKE0, SDCLK, SDNCAS, SDNE0, SDNRAS, SDNWE
        p.PC3,  // FMC_SDCKE0
        p.PG8,  // FMC_SDCLK
        p.PG15, // FMC_SDNCAS
        p.PC2,  // FMC_SDNE0
        p.PF11, // FMC_SDNRAS
        p.PH5,  // FMC_SDNWE
        is42s32400f_6::MyIs42s32400f6 {},
    );

    let mut delay = Delay;

    let ram_slice = unsafe {
        // Initialise controller and SDRAM
        let ram_ptr: *mut u16 = sdram.init(&mut delay) as *mut _;

        // Convert raw pointer to slice
        core::slice::from_raw_parts_mut(ram_ptr, sdram_size / core::mem::size_of::<u16>())
    };

    const RK043FN48H_HSYNC: u16 = 30; // Horizontal synchronization
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
        data_enable_polarity: PolarityActive::ActiveHigh,
        pixel_clock_polarity: PolarityEdge::RisingEdge,
    };

    info!("Creating LTDC");
    let mut ltdc_de = Output::new(p.PK7, Level::Low, Speed::High);
    let mut ltdc_disp_ctrl = Output::new(p.PI12, Level::Low, Speed::High);
    let mut ltdc_bl_ctrl = Output::new(p.PK3, Level::Low, Speed::High);
    let mut ltdc = Ltdc::new_with_pins(
        p.LTDC, Irqs, p.PI14, p.PI10, p.PI9, p.PE4, p.PJ13, p.PJ14, p.PJ15, p.PG12, p.PK4, p.PK5,
        p.PK6, p.PJ7, p.PJ8, p.PJ9, p.PJ10, p.PJ11, p.PK0, p.PK1, p.PK2, p.PI15, p.PJ0, p.PJ1,
        p.PJ2, p.PJ3, p.PJ4, p.PJ5, p.PJ6,
    );
    info!("Initializing LTDC");
    ltdc.init(&ltdc_config);
    ltdc_de.set_low();
    ltdc_bl_ctrl.set_high();
    ltdc_disp_ctrl.set_high();

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
    // for i in 0..DISPLAY_WIDTH * DISPLAY_HEIGHT {
    // ram_slice[i] = (i as u32) | 0xFF000000;
    // ram_slice[i] = 0xFF669933;
    // }
    for x in 0..DISPLAY_WIDTH {
        for y in 0..DISPLAY_HEIGHT {
            if x < 30 {
                ram_slice[y * DISPLAY_WIDTH + x] = 0b11111_000000_00000;
            } else {
                ram_slice[y * DISPLAY_WIDTH + x] = 0b00000_111111_00000;
            }
        }
    }
    // Put blue dots in the corners
    ram_slice[0] = 0b00000_000000_11111;
    ram_slice[DISPLAY_WIDTH - 1] = 0b00000_000000_11111;
    ram_slice[(DISPLAY_HEIGHT - 1) * DISPLAY_WIDTH] = 0b00000_000000_11111;
    ram_slice[DISPLAY_HEIGHT * DISPLAY_WIDTH - 1] = 0b00000_000000_11111;
    // ram_slice[0] = 0xFFFF0000;

    println!("After filling framebuffer: {:?}", ram_slice[0..10]);

    info!("setting buffer");
    unwrap!(
        ltdc.set_buffer(LtdcLayer::Layer1, ram_slice.as_ptr() as *const _)
            .await
    );

    info!("done");

    info!("Connecting to I2c");

    let mut i2c = I2c::new(
        p.I2C3,
        p.PH7,
        p.PH8,
        Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        Hertz(100_000),
        Default::default(),
    );

    loop {
        // button.wait_for_any_edge().await;
        // info!("high");
        led.set_high();
        // Timer::after_millis(300).await;
        touchscreen.wait_for_rising_edge().await;

        // button.wait_for_any_edge().await;
        // info!("low");
        led.set_low();
        // Timer::after_millis(300).await;
        touchscreen.wait_for_rising_edge().await;
    }
}

// type TargetPixelType = u16;

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
            divq: Some(PllQDiv::DIV4),
            divr: Some(PllRDiv::DIV5),
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;

        embassy_stm32::init(config)
    }
}

/*

*/
/// ISI IS42S32400F SDRAM
#[allow(unused)]

/// Speed Grade 6
pub mod is42s32400f_6 {
    use stm32_fmc::{SdramChip, SdramConfiguration, SdramTiming};

    const BURST_LENGTH_1: u16 = 0x0000;
    const BURST_LENGTH_2: u16 = 0x0001;
    const BURST_LENGTH_4: u16 = 0x0002;
    const BURST_LENGTH_8: u16 = 0x0004;
    const BURST_TYPE_SEQUENTIAL: u16 = 0x0000;
    const BURST_TYPE_INTERLEAVED: u16 = 0x0008;
    const CAS_LATENCY_2: u16 = 0x0020;
    const CAS_LATENCY_3: u16 = 0x0030;
    const OPERATING_MODE_STANDARD: u16 = 0x0000;
    const WRITEBURST_MODE_PROGRAMMED: u16 = 0x0000;
    const WRITEBURST_MODE_SINGLE: u16 = 0x0200;

    /// Is42s32400f with Speed Grade 6
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct MyIs42s32400f6 {}

    impl SdramChip for MyIs42s32400f6 {
        /// Value of the mode register
        const MODE_REGISTER: u16 = BURST_LENGTH_1
            | BURST_TYPE_SEQUENTIAL
            | CAS_LATENCY_2
            | OPERATING_MODE_STANDARD
            | WRITEBURST_MODE_SINGLE;

        /// Timing Parameters
        const TIMING: SdramTiming = SdramTiming {
            startup_delay_ns: 100_000,    // 100 µs
            max_sd_clock_hz: 100_000_000, // 100 MHz
            refresh_period_ns: 15_625,    // 64ms / (4096 rows) = 15625ns
            mode_register_to_active: 2,   // tMRD = 2 cycles
            exit_self_refresh: 7,         // tXSR = 70ns
            active_to_precharge: 4,       // tRAS = 42ns
            row_cycle: 7,                 // tRC = 60ns
            row_precharge: 2,             // tRP = 18ns
            row_to_column: 2,             // tRCD = 18ns
        };

        /// SDRAM controller configuration
        const CONFIG: SdramConfiguration = SdramConfiguration {
            column_bits: 8,
            row_bits: 12,
            memory_data_width: 16, // 32-bit
            internal_banks: 4,     // 4 internal banks
            cas_latency: 2,        // CAS latency = 3
            write_protection: false,
            read_burst: true,
            read_pipe_delay_cycles: 0,
        };
    }
}
