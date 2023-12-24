#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <cmsis/cmsis_device.h>

void __initialize_hardware_early(void);
void Reset_Handler(void)
{
	__initialize_hardware_early();
	__cmsis_start();
}

void _exit(int code)
{
	(void) code;
	__asm__ ("BKPT");
	while (1);
}
