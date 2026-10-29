# Adds the new source file and refreshes the bridge cell after the configuration-frame update.
# Run in the Vivado Tcl console with the project open:  source D:/internship/axi4_block_canfd/files/apply_cfg_frame_update.tcl
add_files -norecurse D:/internship/axi4_block_canfd/files/can_cfg_regs.v
update_compile_order -fileset sources_1
open_bd_design [get_files axi_block.bd]
update_module_reference [get_ips axi_block_can_fd_usb_top_0_2]
# the bridge cell now has a CFG_FRAME_EN parameter (default 1 = wait for the configuration frame)
validate_bd_design
save_bd_design
