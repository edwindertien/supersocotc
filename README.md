# supersocotc
controller conversion tools and repair of a SuperSocoTC bike

inspired by / adapted from https://github.com/stprograms/SuperSoco485Monitor and https://github.com/stprograms/SuperSoco485

goal: create RS485 driver for instrument panel of supersoco when replacing motor driver does not create these signals... read SOC of battery 

why replacement driver: enable regenerative braking!

(first problem tackled: motor didn't work anymore: hall-effect sensor in Bosch motor broken)


next up: setup is not with ECU but instrument panel, battery and motor control (RS485)

fardriver app control not working.. try to build python sketch  based on 
https://github.com/jackhumbert/fardriver-controllers/tree/main
https://github.com/bobecek79/ESP32-Fardriver-BLE-Reader

