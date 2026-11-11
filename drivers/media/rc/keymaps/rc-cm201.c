// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CM201-2（sunniwell）遥控器键表。
 *
 * 2026-11-08 依据原厂 Android /etc/key.xml（sunniwell-key 段，原始码以 dd22 结尾）重建。8 位命令是 NEC scancode 的低字节；下表已与实机 evdev 抓取结果交叉核对（方向键、OK、返回、菜单、主页、静音、设置、电源）。
 *
 * 菜单、频道加减、主页与四个颜色键（原厂 F6–F9）共 8 项不用原厂键名，换成 Kodi 认得的键，行尾注出原厂键名：Kodi 经 libinput 与 xkbcommon 把键码翻成 keysym 再查键表，原厂的这几个键名在那里不起作用或语义不对。换法取自设备目录 tools/stock-ir-keymap.py 的 KODI_REMAP（依据是同款遥控器在 cm211-1 上的 Kodi 实测），与 rootfs overlay 的 /etc/rc_keymaps/rc-cm201.toml 逐项相同。
 *
 * Copyright (c) 2017 HiSilicon Technologies Co., Ltd.
 */

#include <linux/module.h>
#include <media/rc-map.h>

static struct rc_map_table cm201_keymap[] = {
	{ 0x0000225f, KEY_BACKSPACE},
	{ 0x00002260, KEY_F24},
	{ 0x00002261, KEY_F23},
	{ 0x00002263, KEY_BACKSPACE},
	{ 0x00002280, KEY_VOLUMEUP},
	{ 0x00002281, KEY_VOLUMEDOWN},
	{ 0x00002282, KEY_COMPOSE},	/* 原厂 KEY_MENU */
	{ 0x00002283, KEY_YELLOW},	/* 原厂 KEY_F8 */
	{ 0x00002284, KEY_FORWARD},
	{ 0x00002285, KEY_PAGEUP},	/* 原厂 KEY_CHANNELUP */
	{ 0x00002286, KEY_PAGEDOWN},	/* 原厂 KEY_CHANNELDOWN */
	{ 0x00002287, KEY_0},
	{ 0x00002288, KEY_HOMEPAGE},	/* 原厂 KEY_HOME */
	{ 0x00002289, KEY_REWIND},
	{ 0x0000228a, KEY_7},
	{ 0x0000228b, KEY_8},
	{ 0x0000228c, KEY_PAGEDOWN},
	{ 0x0000228d, KEY_SETUP},
	{ 0x0000228e, KEY_4},
	{ 0x0000228f, KEY_5},
	{ 0x00002291, KEY_GREEN},	/* 原厂 KEY_F7 */
	{ 0x00002292, KEY_1},
	{ 0x00002293, KEY_2},
	{ 0x00002295, KEY_BACK},
	{ 0x00002296, KEY_STOP},
	{ 0x00002298, KEY_AUDIO},
	{ 0x00002299, KEY_LEFT},
	{ 0x0000229c, KEY_MUTE},
	{ 0x000022c0, KEY_BACKSPACE},
	{ 0x000022c1, KEY_RIGHT},
	{ 0x000022c3, KEY_BLUE},	/* 原厂 KEY_F9 */
	{ 0x000022c4, KEY_9},
	{ 0x000022c5, KEY_INFO},
	{ 0x000022c8, KEY_6},
	{ 0x000022ca, KEY_UP},
	{ 0x000022cc, KEY_3},
	{ 0x000022cd, KEY_RED},	/* 原厂 KEY_F6 */
	{ 0x000022ce, KEY_ENTER},
	{ 0x000022d0, KEY_BACKSPACE},
	{ 0x000022d2, KEY_DOWN},
	{ 0x000022d6, KEY_FN_B},
	{ 0x000022d9, KEY_PLAYPAUSE},
	{ 0x000022da, KEY_FN_1},
	{ 0x000022dc, KEY_POWER},
	{ 0x000022dd, KEY_PAGEUP},
	{ 0x000022f0, KEY_PHONE},
};

static struct rc_map_list cm201_map = {
	.map = {
		.scan	  = cm201_keymap,
		.size	  = ARRAY_SIZE(cm201_keymap),
		.rc_proto = RC_PROTO_NEC,
		.name	  = "rc-cm201",
	}
};

static int __init init_rc_map_cm201(void)
{
	return rc_map_register(&cm201_map);
}

static void __exit exit_rc_map_cm201(void)
{
	rc_map_unregister(&cm201_map);
}

module_init(init_rc_map_cm201)
module_exit(exit_rc_map_cm201)

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CM201-2 remote controller keytable");
