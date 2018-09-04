"""
DebugScript for SAMB devices.
If programming mode is set to managed by script it will debug out of ram.
"""
debugOutOfRam = False
patchBreakPointId = -1
appEntryBreakPointId = -1
postFlashDownloadAddr = 0x1236
entryPointAdr = 0

def should_process_breakpoint(api, adr, bpId, obj):
	global patchBreakPointId, appEntryBreakPointId, entryPointAdr,debugOutOfRam
	api.Print("Break at " + hex(adr), "DebugScript")

	if adr == postFlashDownloadAddr:
		if patchBreakPointId != -1:
			api.DelBp(patchBreakPointId)
			patchBreakPointId = -1
		appEntryBreakPointId = api.CreateBpAtAddress(entryPointAdr)			# Break app entry to initialize C runtime
		api.Write32(0x10041FC8,1)			# keep debugger clock running
		if debugOutOfRam:
			return HandlePatchBreakpoint(api)
		else:
			return False
	elif adr == entryPointAdr:
		return HandleAppEntryBreakpoint(api)
	return True

def on_reset(api,resetAdr):
	global patchBreakPointId, entryPointAdr
	entryPointAdr = api.CalcNumericValue("app_entry", 0)
	if entryPointAdr < 0x10009000:
		api.Print("Unable to find entry point address.","DebugScript")
		api.DisplayDialogBox("Unable to setup device", "Invalid entry point address:\n" +hex(entryPointAdr),7)
		return False
	api.Write32(0x4000F040,0x00)				# Trigger cold boot
	api.Write32(0x4000F044,0x12345678)			# 
	
	patchBreakPointId = api.CreateBpAtAddress(postFlashDownloadAddr)	# Break after SPI flash has been copied


def on_launch(api, debugProps):
	global debugOutOfRam
	debugProps["RequireHardReset"] = True
	if "EraseRule" in debugProps:
		if debugProps["EraseRule"] == 7:
			debugOutOfRam = True
			debugProps["EraseRule"] = 2

	return debugProps

def HandleAppEntryBreakpoint(api):
	global appEntryBreakPointId
	api.Print("Disabling watchdog and enabling ARM debug.","DebugScript")
	api.Write32(0x10041FC8, 1)				# Enabled ARM debug through SWD (only for debug mode)
	api.Write32(0x40008008, 0x00000000); 	# Disable watchdog
	api.Write32(0x40000000, 0x00000000); 	# Disable watchdog
	if appEntryBreakPointId != -1:
		api.DelBp(appEntryBreakPointId)
		appEntryBreakPointId = -1
		return False
	return True

def HandlePatchBreakpoint(api):
	global patchBreakPointId

	patchPath = api.ExpandMacro("$(PROJECT_BLEAPP_ROOT)\patches\patch.hex")
	api.Print("Loading firmware patch from:\n"+patchPath ,"DebugScript")
	if not api.LoadFile(patchPath,0,"base"):
		api.Print("Unable to load firmware patch from:\n"+patchPath ,"DebugScript")
		api.DisplayDialogBox("Unable to setup device", "Unable to load firmware patch from path:\n" +patchPath,7)
		return True

	appPath = api.ExpandMacro("$(OutputDirectory)\\$(OutputFileName).hex")
	api.Print("Loading application from:\n"+appPath ,"DebugScript")
	if not api.LoadFile(appPath,0,"base"):
		api.Print("Unable to load application from:\n"+appPath,"DebugScript")
		api.DisplayDialogBox("Unable to setup device", "Unable to load application:\n" +appPath,7)
		return True
	
	api.Print("Patching rom values" ,"DebugScript")
	api.Write32(0x1004000c,0x100056e9)	# rw_main (Function)
	api.Write32(0x10040010,0x10005851)	# rwip_eif_get (Function)
	api.Write32(0x1004004c,0x10005901)	# XTAL_mode_check (Function)
	api.Write32(0x10040034,0x10005a09)	# system_sleep (Function)
	api.Write32(0x10040068,0x10005e9d)	# rwble_isr (Function)
	api.Write32(0x10040088,0x10006011)	# lld_evt_schedule (Function)
	api.Write32(0x100400bc,0x100061b1)	# gapm_get_task_from_id (Function)
	api.Write32(0x100400e4,0x100065ed)	# ke_msg_send (Function)
	api.Write32(0x100400e8,0x100069d3)	# rf_init (Function)
	api.Write32(0x100400ec,0x10006a3d)	# ulp_rf_init (Function)
	api.Write32(0x1004010c,0x10006b99)	# rwip_sleep (Function)
	api.Write32(0x10040154,0x10006e39)	# intc_init (Function)	
	api.Write32(0x10040150,0x10006ed3)	# aon_sleep_timer_irq_fn (Function)
	api.Write32(0x10040160,0x10006f11)	# uart_init (Function)
	api.Write32(0x10040174,0x1000708d)	# ulp_wakeup (Function)
	api.Write32(0x10040210,0x10007b5d)	# clkClb_informAction (Function)
	api.Write32(0x10040038,0x10005be7)	# early_entry (Function)
	api.Write32(0x1000003C,0x10005bbb)	# 0x1000003C (Address)	

	# api.Write32(0x10007b88,0x00000001)	# gu32_override_xo_cap_values (Variable)
	api.Write32(0x10008234,0x00000001)
	# api.Write32(0x10007b8c,0x00000004)	# gu32xo_cap_values (Variable)
	api.Write32(0x10008238,0x00000004)

	api.Write32(0x10040168,0x00000003)	# ram_keep_awake (Variable)
	api.Write32(0x100400d9,0x00000001)	# gtl_eif_type (Variable)
	api.Write32(0x100400dc,0x00019831)  # 0x100400dc (Address)
	# api.Write32(0x10007abc,0x28000000)	# gu32fw_version (Variable)
	api.Write32(0x10040000,0x00000001)	# gu8patching_done (Variable)

	stackTuple = GetStackAddrAndSize(api, api.ExpandMacro("$(OutputDirectory)\\$(OutputFileName).map"))
	if stackTuple == (0,0):
		api.Print("Unable to find app stack patch address.","DebugScript")
	else:
		api.Print("app stack address is " + hex(stackTuple[0]) + " with size " + hex(stackTuple[1]), "DebugScript")
		api.Write32(0x10040028, stackTuple[0])
		api.Write32(0x1004002c, stackTuple[1])
		api.Write8(0x1004001c, 1)

	api.Write32(0x10040018, 0x05)
	api.Write32(0x10040030, entryPointAdr+1)
	
	return False

def GetStackAddrAndSize(api,mapFile):
	stackStart = 0
	stackEnd = 0
	with open(mapFile, 'r') as inF:
		for line in inF:
			if '_sstack' in line:
				stackStart = int(line.split()[0],0)
			if '_estack' in line:
				stackEnd = int(line.split()[0],0)
			if(stackStart != 0 and stackEnd != 0):
				break
	return (stackStart, stackEnd-stackStart)
