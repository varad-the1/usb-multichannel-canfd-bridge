# Removes axi_vip_0, axi_smc and axi_gpio_1 from the axi_block design. The custom bridge (can_fd_usb_top_0)
# becomes the only AXI master and talks straight to the CAN FD core, one AXI-Lite port per channel.
# Run in the Vivado Tcl console with the project open:  source D:/internship/axi4_block_canfd/files/remove_vip.tcl
open_bd_design [get_files axi_block.bd]
update_module_reference [get_ips axi_block_can_fd_usb_top_0_2]

delete_bd_objs [get_bd_cells axi_vip_0]
delete_bd_objs [get_bd_cells axi_smc]
delete_bd_objs [get_bd_cells axi_gpio_1]
delete_bd_objs [get_bd_intf_ports -quiet gpio_rtl_0]

# channel 0 of the bridge -> CAN FD core, directly
connect_bd_intf_net [get_bd_intf_pins can_fd_usb_top_0/m_axi_0] [get_bd_intf_pins canfd_0/CAN_S_AXI_LITE]

# the bridge's BASE0 parameter is 0x44A00000
assign_bd_address
set_property offset 0x44A00000 [get_bd_addr_segs can_fd_usb_top_0/m_axi_0/SEG_canfd_0_Reg]
validate_bd_design
save_bd_design
