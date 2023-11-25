# Protocol etc

## Received from board

### Voltage/current
VC:addres;v1;i1;v2;i2;v3;i3;v4;i3
* VC -> Identifier for voltage/current data
* Address of the PAC1954, fe. 0x1D
* v1 till v4: the raw voltage reading for ch1 till 4
* i1 till i4: the raw current reading for ch1 till 4

**Formulas**

* Voltage: voltage[V] = v1*(32/65535)
* Current: 
  * FSR: 0,100V/Rsense 
  * current[A] = i1*(FSR/65535)

### Accumulator
AC;ch;raw acc;raw acc count
* AC -> identifier for accumulator data
* ch -> the channel
* raw acc -> raw hex accummulator data 54 bits
* raw acc count -> raw hex accummulator counts 32 bits

**Formulas**
* Energy: (acc/denominator)*PowerFSR*(1/samplerate)*(1/3600)
  * denominator: 2^30
  * Rsense: 25mOhm
  * PowerFSR: 3.2/Rsense = 3.2/0.025 = 128
  * Samplerate: 1024SPS 
  * energy = 128*(i2/2^30)*(1/1024)*(1/3600) [Wh]
           = (i2/2^30)*(1/28800) [Wh]