# Image_Proc

23年11月20日
将十字补线增加标志位cross_flag
当flag等于0时，不补线
当flag等于1时,找到下拐点,且下拐点很近，开始补线
当flag等于2时,只有上拐点，用上拐点的斜率补线，当上拐点消失的时候,清空标志位
未完成的问题
右上拐点会找不到,需要修改八领域算法,当找到右下拐点并标志位符合条件时,生长先向上一直到突变的位置
