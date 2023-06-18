instaltion:
per hand:
just copy my github https://github.com/akadon/Marlin_N32 into https://github.com/MarlinFirmware/Marlin.
  
Linux:     
git clone https://github.com/MarlinFirmware/Marlin .   
git checkout 441416728cd7f0e9b6ebf94f895d1d27fe59d25a # latest commit at this time (tested works)   
git clone https://github.com/akadon/Marlin_N32 Marlin_N32   
cp -r !(.git/\*) Marlin_N32/\* .    
  
Dont forget to change platform to STM32F103RE_creality_maple  

picture guide:

![guide1](1.png "Picture Guide 1")
![guide2](2.png "Picture Guide 2")
![guide3](3.png "Picture Guide 3")

