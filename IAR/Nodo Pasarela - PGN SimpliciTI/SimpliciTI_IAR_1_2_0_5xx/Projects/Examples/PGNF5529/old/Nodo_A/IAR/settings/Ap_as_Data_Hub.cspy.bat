@REM This batch file has been generated by the IAR Embedded Workbench
@REM C-SPY Debugger, as an aid to preparing a command line for running
@REM the cspybat command line utility using the appropriate settings.
@REM
@REM You can launch cspybat by typing the name of this batch file followed
@REM by the name of the debug file (usually an ELF/DWARF or UBROF file).
@REM Note that this file is generated every time a new debug session
@REM is initialized, so you may want to move or rename the file before
@REM making changes.
@REM 


"C:\Program Files (x86)\IAR Systems\Embedded Workbench 6.0\common\bin\cspybat" "C:\Program Files (x86)\IAR Systems\Embedded Workbench 6.0\430\bin\430proc.dll" "C:\Program Files (x86)\IAR Systems\Embedded Workbench 6.0\430\bin\430fet.dll"  %1 --plugin "C:\Program Files (x86)\IAR Systems\Embedded Workbench 6.0\430\bin\430bat.dll" --backend -B "--hardware_multiplier" "32" "--hwmult_type" "8" "-p" "C:\Program Files (x86)\IAR Systems\Embedded Workbench 6.0\430\config\MSP430F5529.ddf" "--core=430Xv2" "--data_model=small" "--iv_base" "0xFF80" "--no_wrap_around" "--cpu_bug_30" "-d" "fet" "--verify_all" "--erase_main" "--set_putchar_breakpoint" "--set_getchar_breakpoint" "--derivative" "MSP430F5529" "--protocol" "automatic" "--eem" "EMEX_LARGE_5XX" "--port" "Automatic" "--connection" "ti_usb" "--settlingtime=0" "--msp430_dll" "msp430.dll" "--vccDefault" "3.3" 


