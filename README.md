# pandora
原子哥物联网开发板，自学习
## 主要用于学习一下hal库和stm32cube，没有什么参考价值。大家可以越过了，我就为公司和私人电脑方便拉取管理一下。 ##
### 顺便说下个人的结论吧，stm32cube这个东西对于初学者来说确实比较爽，省的写驱动部分了，只需要配置下就可以了，但是我这人前几年一直写的寄存器，也就今年才用库函数，所以还是不太习惯，感觉有了问题不知道找哪？比较懵逼。当然目前用着这个东西还是挺给力的，只发现了一个bug，也可能是我的问题，仅供参考吧：
### 在时钟部分设置为外部时钟时，PLL第一个分频器 PLLM总是不会给我初始化，少这句RCC_OscInitStruct.PLL.PLLM = 1，导致时钟异常的问题。
