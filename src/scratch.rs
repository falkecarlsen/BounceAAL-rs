println!("Starting I2C self test");


let mut i2c_slave = i2c_slave_init(
peripherals.i2c1,
peripherals.pins.gpio18.into(),
peripherals.pins.gpio19.into(),
SLAVE_BUFFER_SIZE,
SLAVE_ADDR,
)?;

let tx_buf: [u8; 8] = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef];

println!("-------- TESTING SIMPLE MASTER WRITE --------");
i2c_master.write(SLAVE_ADDR, &tx_buf, BLOCK)?;

let mut rx_buf: [u8; 8] = [0; 8];
match i2c_slave.read(&mut rx_buf, BLOCK) {
Ok(_) => println!("Master send {:?} Slave receives {:?}", tx_buf, rx_buf),
Err(e) => println!("Error: {:?}", e),
}

println!("-------- TESTING SIMPLE MASTER READ --------");
i2c_slave.write(&tx_buf, BLOCK)?; // fill slave buffer before master tries to read

let mut rx_buf: [u8; 8] = [0; 8];
match i2c_master.read(SLAVE_ADDR, &mut rx_buf, BLOCK) {
Ok(_) => println!("Slave send {:?} Master receives {:?}", tx_buf, rx_buf),
Err(e) => println!("Error: {:?}", e),
}

println!("-------- TESTING READ/WRITE REGISTER --------");

let thread0 = std::thread::Builder::new()
.stack_size(7000)
.spawn(move || {
let mut data: [u8; 256] = [0; 256];
loop {
let mut reg_addr: [u8; 1] = [0];
let res = i2c_slave.read(&mut reg_addr, BLOCK);
if res.is_err() {
println!(
"SLAVE: failed to read register address from master: Error: {:?}",
res
);
continue;
}
let mut rx_data: [u8; 1] = [0];
match i2c_slave.read(&mut rx_data, 0) {
Ok(_) => {
println!(
"SLAVE: write operation {:#04x} to reg addr {:#04x}",
rx_data[0], reg_addr[0]
);
data[reg_addr[0] as usize] = rx_data[0];
}
Err(_) => {
let d = data[reg_addr[0] as usize];
println!(
"SLAVE: read operation {:#04x} from reg addr {:#04x}",
d, reg_addr[0]
);
i2c_slave.write(&[d], BLOCK).unwrap();
}
}
}
})?;



// allow thread to run
std::thread::yield_now();

let reg_addr: u8 = 0x05;
let new_value: u8 = 0x42;

println!("MASTER: read reg addr {:#04x}", reg_addr);
let mut rx_buf: [u8; 1] = [0; 1];
// TODO: make write_read work
i2c_master.write(SLAVE_ADDR, &[reg_addr], BLOCK)?;
i2c_master.read(SLAVE_ADDR, &mut rx_buf, BLOCK)?;
println!(
    "MASTER: value of reg addr {:#04x} is {:#04x}",
    reg_addr, rx_buf[0]
);

println!("---------------------");

println!(
    "MASTER: write {:#04x} to reg addr {:#04x}",
    new_value, reg_addr
);
i2c_master.write(SLAVE_ADDR, &[reg_addr, new_value], BLOCK)?;

println!("---------------------");

println!("MASTER: read reg addr {:#04x}", reg_addr);
let mut rx_buf: [u8; 1] = [0; 1];
// TODO: make write_read work
i2c_master.write(SLAVE_ADDR, &[reg_addr], BLOCK)?;
i2c_master.read(SLAVE_ADDR, &mut rx_buf, BLOCK)?;
println!(
    "MASTER: value of reg addr {:#04x} is {:#04x}",
    reg_addr, rx_buf[0]
);

thread0.join().unwrap();
Ok(())