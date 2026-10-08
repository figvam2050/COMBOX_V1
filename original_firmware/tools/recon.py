import struct, collections
p="/Users/vlad/Documents/PlatformIO/Projects/COMBOX_Vision_Deye_fixed_project_files/combobox_bkp_40000.bin"
d=open(p,'rb').read()
print("size",len(d))
def w(o): return struct.unpack_from('<I',d,o)[0]
# vector tables
for base in (0,0x4000,0x8000,0x10000):
    if base+8<=len(d):
        sp,rv=w(base),w(base+4)
        print("VT @ file+0x%05X: SP=0x%08X RESET=0x%08X"%(base,sp,rv))
# flash end (erased 0xFF)
end=len(d)
while end>0 and d[end-1]==0xFF: end-=1
print("last non-FF offset: 0x%X (%d)"%(end,end))
# peripheral words
periph=collections.Counter(); locs=collections.defaultdict(list)
for o in range(0,len(d)-3,4):
    v=w(o)
    if 0x40000000<=v<0x40030000 or 0xE0000000<=v<0xE0100000:
        periph[v]+=1
        if len(locs[v])<8: locs[v].append(o)
names={0x40013800:'USART1/SPI1?',0x40013000:'ADC1?',0x40004400:'USART2',0x40004800:'USART3',0x40004C00:'UART4',0x40005000:'UART5',
0x40006400:'CAN1',0x40006800:'CAN2',0x40010000:'CAN1remap?',0x40010800:'GPIOA',0x40010C00:'GPIOB',0x40011000:'GPIOC',0x40011400:'GPIOD',0x40011800:'GPIOE',
0x40021000:'RCC',0x40007000:'BKP/RCC?',0x40006C00:'RTC?',0x40005400:'?',0x40000000:'TIM2',0x40000400:'TIM3',0x40000800:'TIM4',0x40001000:'TIM6',0x40001400:'TIM7',
0x40001800:'TIM12',0x40001C00:'TIM13',0x40002000:'TIM14',0x40002800:'SPI2/I2S2',0x40003000:'IWDG',0x40003400:'WWDG',0x40003800:'I2C1',0x40003C00:'I2C2',0x40005C00:'DAC',
0x40010400:'AFIO',0x40010000:'EXTI',0x40012400:'SPI1',0x40012C00:'ADC2',0x40011C00:'USART1@',0x40020000:'DMA1',0x40020400:'DMA2'}
for v,c in sorted(periph.items(), key=lambda x:-x[1])[:60]:
    print("0x%08X x%-3d %-14s locs(file offs): %s"%(v,c,names.get(v,''),['0x%X'%x for x in locs[v]]))
