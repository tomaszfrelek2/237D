#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const char ____versions[]
__used __section("__versions") =
	"\x1c\x00\x00\x00\x7b\xcc\x27\x84"
	"_raw_spin_lock_irq\0\0"
	"\x20\x00\x00\x00\x53\x0f\x75\x4b"
	"_raw_spin_unlock_irq\0\0\0\0"
	"\x18\x00\x00\x00\x07\xa7\x43\x32"
	"usb_control_msg\0"
	"\x1c\x00\x00\x00\x3e\xc2\x4d\xa1"
	"__dynamic_dev_dbg\0\0\0"
	"\x1c\x00\x00\x00\xe3\xe9\xa9\x61"
	"__tty_alloc_driver\0\0"
	"\x18\x00\x00\x00\xc1\x7e\xb2\x67"
	"tty_std_termios\0"
	"\x1c\x00\x00\x00\xfd\x9c\xb1\x91"
	"tty_register_driver\0"
	"\x1c\x00\x00\x00\x21\x5b\xb6\x60"
	"usb_register_driver\0"
	"\x10\x00\x00\x00\x7e\x3a\x2c\x12"
	"_printk\0"
	"\x20\x00\x00\x00\x2d\x5f\xdb\x7d"
	"tty_unregister_driver\0\0\0"
	"\x1c\x00\x00\x00\x53\x5d\x99\x78"
	"tty_driver_kref_put\0"
	"\x1c\x00\x00\x00\xef\x6d\x5c\xa6"
	"alt_cb_patch_nops\0\0\0"
	"\x18\x00\x00\x00\x04\x2b\x8d\x2d"
	"usb_submit_urb\0\0"
	"\x14\x00\x00\x00\xc5\x23\xf6\xbf"
	"_dev_err\0\0\0\0"
	"\x28\x00\x00\x00\xb3\x1c\xa2\x87"
	"__ubsan_handle_out_of_bounds\0\0\0\0"
	"\x28\x00\x00\x00\xa4\x92\xf5\xf7"
	"usb_autopm_put_interface_async\0\0"
	"\x18\x00\x00\x00\x36\x21\xee\x14"
	"usb_kill_urb\0\0\0\0"
	"\x1c\x00\x00\x00\xfe\x2d\xc1\x03"
	"cancel_work_sync\0\0\0\0"
	"\x18\x00\x00\x00\x64\xbd\x8f\xba"
	"_raw_spin_lock\0\0"
	"\x1c\x00\x00\x00\x34\x4b\xb5\xb5"
	"_raw_spin_unlock\0\0\0\0"
	"\x18\x00\x00\x00\xca\x54\xfb\xaa"
	"tty_port_put\0\0\0\0"
	"\x10\x00\x00\x00\xfd\xf9\x3f\x3c"
	"sprintf\0"
	"\x20\x00\x00\x00\x6f\x36\xbe\x6e"
	"ktime_get_mono_fast_ns\0\0"
	"\x1c\x00\x00\x00\x90\x00\xac\xb8"
	"tty_port_tty_hangup\0"
	"\x1c\x00\x00\x00\x51\x18\x30\xf2"
	"tty_port_tty_wakeup\0"
	"\x18\x00\x00\x00\xc9\x3d\x29\x6e"
	"tty_port_hangup\0"
	"\x18\x00\x00\x00\x72\x98\xfd\xd7"
	"tty_port_close\0\0"
	"\x18\x00\x00\x00\x83\x28\x36\x59"
	"usb_deregister\0\0"
	"\x20\x00\x00\x00\x0b\x05\xdb\x34"
	"_raw_spin_lock_irqsave\0\0"
	"\x24\x00\x00\x00\x70\xce\x5c\xd3"
	"_raw_spin_unlock_irqrestore\0"
	"\x2c\x00\x00\x00\xc6\xfa\xb1\x54"
	"__ubsan_handle_load_invalid_value\0\0\0"
	"\x10\x00\x00\x00\x7e\xa4\x29\x48"
	"memcpy\0\0"
	"\x28\x00\x00\x00\x97\x0b\x4e\xc0"
	"usb_autopm_get_interface_async\0\0"
	"\x28\x00\x00\x00\xb0\xa7\x15\xda"
	"__tty_insert_flip_string_flags\0\0"
	"\x20\x00\x00\x00\xbe\x8f\x0d\x73"
	"tty_flip_buffer_push\0\0\0\0"
	"\x1c\x00\x00\x00\xcb\xf6\xfd\xf0"
	"__stack_chk_fail\0\0\0\0"
	"\x14\x00\x00\x00\x4b\x8d\xfa\x4d"
	"mutex_lock\0\0"
	"\x18\x00\x00\x00\x38\xf0\x13\x32"
	"mutex_unlock\0\0\0\0"
	"\x20\x00\x00\x00\xfb\x4c\xfe\xe4"
	"tty_standard_install\0\0\0\0"
	"\x20\x00\x00\x00\x5f\x69\x96\x02"
	"refcount_warn_saturate\0\0"
	"\x24\x00\x00\x00\x52\x3f\x0a\x4b"
	"gic_nonsecure_priorities\0\0\0\0"
	"\x14\x00\x00\x00\xd3\x85\x33\x2d"
	"system_wq\0\0\0"
	"\x18\x00\x00\x00\x36\xf2\xb6\xc5"
	"queue_work_on\0\0\0"
	"\x18\x00\x00\x00\x5c\x2d\x1b\x7b"
	"usb_put_intf\0\0\0\0"
	"\x10\x00\x00\x00\xba\x0c\x7a\x03"
	"kfree\0\0\0"
	"\x18\x00\x00\x00\xa9\xec\x3e\xbf"
	"gpiochip_remove\0"
	"\x1c\x00\x00\x00\xa7\xa0\xd7\x37"
	"device_remove_file\0\0"
	"\x1c\x00\x00\x00\x7f\x45\x42\xc8"
	"tty_port_tty_get\0\0\0\0"
	"\x14\x00\x00\x00\xb0\x2a\xd4\xc5"
	"tty_vhangup\0"
	"\x18\x00\x00\x00\x87\x5f\x82\x6e"
	"tty_kref_put\0\0\0\0"
	"\x20\x00\x00\x00\xc6\x6c\xe9\xbf"
	"tty_unregister_device\0\0\0"
	"\x18\x00\x00\x00\x09\xd5\x95\xa8"
	"usb_free_urb\0\0\0\0"
	"\x1c\x00\x00\x00\x97\xfb\xd1\x71"
	"usb_free_coherent\0\0\0"
	"\x28\x00\x00\x00\xc6\x99\xf0\xda"
	"usb_driver_release_interface\0\0\0\0"
	"\x2c\x00\x00\x00\x61\xe5\x48\xa6"
	"__ubsan_handle_shift_out_of_bounds\0\0"
	"\x1c\x00\x00\x00\x63\xa5\x03\x4c"
	"random_kmalloc_seed\0"
	"\x18\x00\x00\x00\xc1\xee\x6a\x22"
	"kmalloc_caches\0\0"
	"\x18\x00\x00\x00\x11\x70\x83\xbf"
	"kmalloc_trace\0\0\0"
	"\x24\x00\x00\x00\x5c\xa9\xf8\x27"
	"usb_autopm_get_interface\0\0\0\0"
	"\x24\x00\x00\x00\x2e\x57\x9a\x38"
	"usb_autopm_put_interface\0\0\0\0"
	"\x18\x00\x00\x00\x3d\x67\x50\xfc"
	"tty_port_open\0\0\0"
	"\x20\x00\x00\x00\xd8\x94\xd3\x0b"
	"tty_termios_baud_rate\0\0\0"
	"\x10\x00\x00\x00\xad\x64\xb7\xdc"
	"memset\0\0"
	"\x20\x00\x00\x00\x28\xe1\xa4\x12"
	"__arch_copy_from_user\0\0\0"
	"\x10\x00\x00\x00\x89\xbc\xcb\xc6"
	"capable\0"
	"\x1c\x00\x00\x00\x54\xfc\xbb\x6c"
	"__arch_copy_to_user\0"
	"\x18\x00\x00\x00\x8c\x89\xd4\xcb"
	"fortify_panic\0\0\0"
	"\x18\x00\x00\x00\xa9\xb0\xc7\xb0"
	"usb_ifnum_to_if\0"
	"\x18\x00\x00\x00\x9f\x0c\xfb\xce"
	"__mutex_init\0\0\0\0"
	"\x18\x00\x00\x00\x37\x9a\x65\xe0"
	"tty_port_init\0\0\0"
	"\x1c\x00\x00\x00\x6a\x7c\x6b\x89"
	"usb_alloc_coherent\0\0"
	"\x18\x00\x00\x00\x2f\xa7\x7a\xde"
	"usb_alloc_urb\0\0\0"
	"\x14\x00\x00\x00\x14\x9d\xa8\xc3"
	"_dev_warn\0\0\0"
	"\x1c\x00\x00\x00\xfd\xb5\x84\xeb"
	"device_create_file\0\0"
	"\x14\x00\x00\x00\x45\x3a\x23\xeb"
	"__kmalloc\0\0\0"
	"\x14\x00\x00\x00\x21\xce\xaf\xea"
	"_dev_info\0\0\0"
	"\x24\x00\x00\x00\xcc\x98\xea\xea"
	"usb_driver_claim_interface\0\0"
	"\x18\x00\x00\x00\xcf\x2d\x09\x5d"
	"usb_get_intf\0\0\0\0"
	"\x24\x00\x00\x00\xc4\x19\x89\xb8"
	"tty_port_register_device\0\0\0\0"
	"\x24\x00\x00\x00\x20\x7a\xbe\xec"
	"gpiochip_add_data_with_key\0\0"
	"\x18\x00\x00\x00\x30\x7d\x8a\xd0"
	"module_layout\0\0\0"
	"\x00\x00\x00\x00\x00\x00\x00\x00";

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v04E2p1410d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1411d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1412d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1414d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1420d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1421d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1422d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1424d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1400d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1401d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1402d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v04E2p1403d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "12A83EDB83FDB6A376B2D96");
