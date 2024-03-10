# Notes

## Pins attribution

We're using Arduino code on a custom ATmega32u4 board, which means the pinout is different, pin numbers are different


|        | Name        | Schema pin | Arduino Name | Arduino pin |
|--------|-------------|------------|--------------|-------------|
| row0   |  PD7 ADC10  | 27         | D6 A7        | 25          |
| row1   |  PB4 ADC11  | 28         | D8 A8        | 26          |
|
| col0   |  PB6 ADC13  | 30         | D10 A10      | 28          |
| col1   |  PC6 OC4A   | 31         | D5           | 5           |
| col2   |  PC7 ICP3   | 32         | D13          | 13          |
| col3   |  PF5 ADC5   | 38         | A2 D20       | 20          |
|
| enc0-0 | PD6 ADC9    | 26         | D12 A11      | 29          |
| enc0-1 | PD4 ADC8    | 25         | D4 A6        | 24          |
| enc0 sw| PD5 XCK1    | 22         | TXLED        |             |
|
| enc1-0 | PF6 ADC6    | 37         | A1 D19       | 19          |
| enc1-1 | PF7 ADC7    | 36         | A0 D18       | 18          |
|
| enc2-0 | PD2 RXD     | 20         | D0           |             |
| enc2-1 | PD3 TXD     | 21         | D1           |             |
|
| enc3-0 | PF1 ADC1    | 40         | A4 D22       | 22          |
| enc3-1 | PF4 ADC4    | 39         | A3 D21       | 21          |
|
| enc4-0 | PB7 PCINT13 | 12         | D11          |             |
| enc4-1 | PB0 PCINT0  | 8          | SS D17       | 17          |
|
| enc5-0 | PE6 INT6    | 1          | D7           |             |
| enc5-1 | PF0 ADC0    | 41         | A5 D23       | 23          |
|
| SDA    | PD1 SDA     | 19         | D2           | 2           |
| SCL    | PD0 SCL     | 18         | D3           | 3           |

## MIDI protocol


| Message               | Status  | Data 1            | Data 2    |
|-----------------------|---------|-------------------|-----------|
| Note Off              | 8n      | Note Number       | Velocity  |
| Note On               | 9n      | Note Number       | Velocity  |
| Polyphonic Aftertouch	| An      | Note Number       | Pressure  |
| Control Change        | Bn      | Controller Number | Data      |
| Program Change        | Cn      | Program Number    | Unused    |
| Channel Aftertouch    | Dn      | Pressure          | Unused    |
| Pitch Wheel           | En      | LSB               | MSB       |


### Debug from MIDI-OX

Note played by Arduino. Note hex 18 = 24, velocity hex 7F = 127

```
 TIMESTAMP IN PORT STATUS DATA1 DATA2 CHAN NOTE EVENT               

 0000058C   1  --     90    18    7F    1  C  1 Note On               
 00000609   1  --     80    18    7F    1  C  1 Note Off              
```

Played from Arturia Minilab mkII, drum pad. Note hex 24 = 36 in decimal

```
 TIMESTAMP IN PORT STATUS DATA1 DATA2 CHAN NOTE EVENT               

 00000779   1  --     99    24    5E   10  C  2 Note On               
 000007BA   1  --     89    24    00   10  C  2 Note Off              
```