avrdude -c usbasp -p atmega64 -U flash:w:Aip.hex -U lfuse:w:0xEF:m -U efuse:w:0xFF:m 
