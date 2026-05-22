#6自由度关节信息
uint8[] joint1
uint8[] joint2
uint8[] joint3
uint8[] joint4
uint8[] joint5
uint8[] joint6 

#图传状态
uint8[] image
uint8 image_state

#气泵状态
uint8 pump_state

#夹爪状态
uint8[] clamp_claw
uint8 clamp_claw_state

#各机构信息（在线或离线）
uint8[] structure_state