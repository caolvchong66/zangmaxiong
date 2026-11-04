
setws {C:/XYS_work/bear_ws}
platform create -name cam_plat -hw {C:/Users/XYS/Desktop/luoji/ov5640_lcd/38_dual_ov5640_lcd/vitis/system_wrapper/hw/system_wrapper.xsa} -proc ps7_cortexa9_0 -os standalone
puts "PLATFORM CREATED"
platform generate
puts "PLATFORM GENERATED"
exit
