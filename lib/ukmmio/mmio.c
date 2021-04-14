#include <stdio.h>
#include <stdlib.h>
#include <uk/libparam.h>

static char *device = "placeholder";
UK_LIB_PARAM_STR(device);

struct pf_device_info2 {
	__u64 size;
	__u64 base_addr;
	unsigned long irq;
	unsigned long dev_id;
};

struct pf_device_info2 pf_dev_info;

/*
 *		[virtio_mmio.]device=<size>@<baseaddr>:<irq>[:<id>]
 *    where:
 *		<size>     := size (can use standard suffixes like K, M or G)
 *		<baseaddr> := physical base address
 *		<irq>      := interrupt number (as passed to request_irq())
 *		<id>       := (optional) platform device id
 */

__u64 parse_until(char *str, char c, int base, char **pEnd) {
	char *p;

	for (p = str; *p && *p != c; p++);
	if (*p) {
		*p = ' ';
	}

	return strtol(str, pEnd, base);
}

void test_mmio() {
	char *pEnd;
	__u64 size, base_addr;
	unsigned long irq, plat_dev_id = 0;
	
	uk_pr_info("mmio.c\n%s\n\n\n", device);

	size = parse_until(device, '@', 0, &pEnd);
	if (!size) {
		// Invalid
	}

	base_addr = parse_until(pEnd, ':', 0, &pEnd);
	if (!base_addr) {
		// Invalid
	}

	irq = parse_until(pEnd, ':', 10, &pEnd);
	if (!irq) {
		// Invalid
	}

	if (*pEnd) {
		plat_dev_id = parse_until(pEnd, 0, 10, NULL);
	}

	pf_dev_info.base_addr = base_addr;
	pf_dev_info.size = size;
	pf_dev_info.irq = irq;
	pf_dev_info.dev_id = plat_dev_id;
}

__u64 mmio_get_base_addr() {
	return pf_dev_info.base_addr;
}
__u64 mmio_get_size() {
	return pf_dev_info.size;
}
unsigned long mmio_get_irq() {
	return pf_dev_info.irq;
}
unsigned long mmio_get_dev_id() {
	return pf_dev_info.dev_id;
}
