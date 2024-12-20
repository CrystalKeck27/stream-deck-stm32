use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::exti::ExtiInput;
use embassy_sync::blocking_mutex::raw::RawMutex;

struct InterruptControlled<'a, M: RawMutex, BUS> {
    exti: ExtiInput<'a>,
    ft5336: Ft5336<'a, M, BUS>,
}

impl<'a, M: RawMutex, BUS> InterruptControlled<'a, M, BUS> {
    pub fn new(exti: ExtiInput<'a>, ft5336: Ft5336<'a, M, BUS>) -> Self {
        Self { exti, ft5336 }
    }
}

impl<'a, M: RawMutex, BUS> InterruptControlled<'a, M, BUS> {
    pub async fn get_event(&mut self) {
        self.exti.wait_for_rising_edge().await;
    }
}

struct Ft5336<'a, M: RawMutex, BUS> {
    device: I2cDevice<'a, M, BUS>,
}

impl<'a, M: RawMutex, BUS> Ft5336<'a, M, BUS> {
    pub fn new(device: I2cDevice<'a, M, BUS>) -> Self {
        Self { device }
    }
}


