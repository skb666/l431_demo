#ifndef __ONCHIP_FLASH_H__
#define __ONCHIP_FLASH_H__

#include <stdint.h>

#define STMFLASH_BASE (0x08000000UL) /*!< FLASH(up to 256 KB) base address */
#define STMFLASH_END (0x08040000UL)  /*!< FLASH END address                */

#define ADDR_FLASH_PAGE_0 (0x08000000UL)   /* Base address of Page 0, 2 Kbytes */
#define ADDR_FLASH_PAGE_1 (0x08000800UL)   /* Base address of Page 1, 2 Kbytes */
#define ADDR_FLASH_PAGE_2 (0x08001000UL)   /* Base address of Page 2, 2 Kbytes */
#define ADDR_FLASH_PAGE_3 (0x08001800UL)   /* Base address of Page 3, 2 Kbytes */
#define ADDR_FLASH_PAGE_4 (0x08002000UL)   /* Base address of Page 4, 2 Kbytes */
#define ADDR_FLASH_PAGE_5 (0x08002800UL)   /* Base address of Page 5, 2 Kbytes */
#define ADDR_FLASH_PAGE_6 (0x08003000UL)   /* Base address of Page 6, 2 Kbytes */
#define ADDR_FLASH_PAGE_7 (0x08003800UL)   /* Base address of Page 7, 2 Kbytes */
#define ADDR_FLASH_PAGE_8 (0x08004000UL)   /* Base address of Page 8, 2 Kbytes */
#define ADDR_FLASH_PAGE_9 (0x08004800UL)   /* Base address of Page 9, 2 Kbytes */
#define ADDR_FLASH_PAGE_10 (0x08005000UL)  /* Base address of Page 10, 2 Kbytes */
#define ADDR_FLASH_PAGE_11 (0x08005800UL)  /* Base address of Page 11, 2 Kbytes */
#define ADDR_FLASH_PAGE_12 (0x08006000UL)  /* Base address of Page 12, 2 Kbytes */
#define ADDR_FLASH_PAGE_13 (0x08006800UL)  /* Base address of Page 13, 2 Kbytes */
#define ADDR_FLASH_PAGE_14 (0x08007000UL)  /* Base address of Page 14, 2 Kbytes */
#define ADDR_FLASH_PAGE_15 (0x08007800UL)  /* Base address of Page 15, 2 Kbytes */
#define ADDR_FLASH_PAGE_16 (0x08008000UL)  /* Base address of Page 16, 2 Kbytes */
#define ADDR_FLASH_PAGE_17 (0x08008800UL)  /* Base address of Page 17, 2 Kbytes */
#define ADDR_FLASH_PAGE_18 (0x08009000UL)  /* Base address of Page 18, 2 Kbytes */
#define ADDR_FLASH_PAGE_19 (0x08009800UL)  /* Base address of Page 19, 2 Kbytes */
#define ADDR_FLASH_PAGE_20 (0x0800A000UL)  /* Base address of Page 20, 2 Kbytes */
#define ADDR_FLASH_PAGE_21 (0x0800A800UL)  /* Base address of Page 21, 2 Kbytes */
#define ADDR_FLASH_PAGE_22 (0x0800B000UL)  /* Base address of Page 22, 2 Kbytes */
#define ADDR_FLASH_PAGE_23 (0x0800B800UL)  /* Base address of Page 23, 2 Kbytes */
#define ADDR_FLASH_PAGE_24 (0x0800C000UL)  /* Base address of Page 24, 2 Kbytes */
#define ADDR_FLASH_PAGE_25 (0x0800C800UL)  /* Base address of Page 25, 2 Kbytes */
#define ADDR_FLASH_PAGE_26 (0x0800D000UL)  /* Base address of Page 26, 2 Kbytes */
#define ADDR_FLASH_PAGE_27 (0x0800D800UL)  /* Base address of Page 27, 2 Kbytes */
#define ADDR_FLASH_PAGE_28 (0x0800E000UL)  /* Base address of Page 28, 2 Kbytes */
#define ADDR_FLASH_PAGE_29 (0x0800E800UL)  /* Base address of Page 29, 2 Kbytes */
#define ADDR_FLASH_PAGE_30 (0x0800F000UL)  /* Base address of Page 30, 2 Kbytes */
#define ADDR_FLASH_PAGE_31 (0x0800F800UL)  /* Base address of Page 31, 2 Kbytes */
#define ADDR_FLASH_PAGE_32 (0x08010000UL)  /* Base address of Page 32, 2 Kbytes */
#define ADDR_FLASH_PAGE_33 (0x08010800UL)  /* Base address of Page 33, 2 Kbytes */
#define ADDR_FLASH_PAGE_34 (0x08011000UL)  /* Base address of Page 34, 2 Kbytes */
#define ADDR_FLASH_PAGE_35 (0x08011800UL)  /* Base address of Page 35, 2 Kbytes */
#define ADDR_FLASH_PAGE_36 (0x08012000UL)  /* Base address of Page 36, 2 Kbytes */
#define ADDR_FLASH_PAGE_37 (0x08012800UL)  /* Base address of Page 37, 2 Kbytes */
#define ADDR_FLASH_PAGE_38 (0x08013000UL)  /* Base address of Page 38, 2 Kbytes */
#define ADDR_FLASH_PAGE_39 (0x08013800UL)  /* Base address of Page 39, 2 Kbytes */
#define ADDR_FLASH_PAGE_40 (0x08014000UL)  /* Base address of Page 40, 2 Kbytes */
#define ADDR_FLASH_PAGE_41 (0x08014800UL)  /* Base address of Page 41, 2 Kbytes */
#define ADDR_FLASH_PAGE_42 (0x08015000UL)  /* Base address of Page 42, 2 Kbytes */
#define ADDR_FLASH_PAGE_43 (0x08015800UL)  /* Base address of Page 43, 2 Kbytes */
#define ADDR_FLASH_PAGE_44 (0x08016000UL)  /* Base address of Page 44, 2 Kbytes */
#define ADDR_FLASH_PAGE_45 (0x08016800UL)  /* Base address of Page 45, 2 Kbytes */
#define ADDR_FLASH_PAGE_46 (0x08017000UL)  /* Base address of Page 46, 2 Kbytes */
#define ADDR_FLASH_PAGE_47 (0x08017800UL)  /* Base address of Page 47, 2 Kbytes */
#define ADDR_FLASH_PAGE_48 (0x08018000UL)  /* Base address of Page 48, 2 Kbytes */
#define ADDR_FLASH_PAGE_49 (0x08018800UL)  /* Base address of Page 49, 2 Kbytes */
#define ADDR_FLASH_PAGE_50 (0x08019000UL)  /* Base address of Page 50, 2 Kbytes */
#define ADDR_FLASH_PAGE_51 (0x08019800UL)  /* Base address of Page 51, 2 Kbytes */
#define ADDR_FLASH_PAGE_52 (0x0801A000UL)  /* Base address of Page 52, 2 Kbytes */
#define ADDR_FLASH_PAGE_53 (0x0801A800UL)  /* Base address of Page 53, 2 Kbytes */
#define ADDR_FLASH_PAGE_54 (0x0801B000UL)  /* Base address of Page 54, 2 Kbytes */
#define ADDR_FLASH_PAGE_55 (0x0801B800UL)  /* Base address of Page 55, 2 Kbytes */
#define ADDR_FLASH_PAGE_56 (0x0801C000UL)  /* Base address of Page 56, 2 Kbytes */
#define ADDR_FLASH_PAGE_57 (0x0801C800UL)  /* Base address of Page 57, 2 Kbytes */
#define ADDR_FLASH_PAGE_58 (0x0801D000UL)  /* Base address of Page 58, 2 Kbytes */
#define ADDR_FLASH_PAGE_59 (0x0801D800UL)  /* Base address of Page 59, 2 Kbytes */
#define ADDR_FLASH_PAGE_60 (0x0801E000UL)  /* Base address of Page 60, 2 Kbytes */
#define ADDR_FLASH_PAGE_61 (0x0801E800UL)  /* Base address of Page 61, 2 Kbytes */
#define ADDR_FLASH_PAGE_62 (0x0801F000UL)  /* Base address of Page 62, 2 Kbytes */
#define ADDR_FLASH_PAGE_63 (0x0801F800UL)  /* Base address of Page 63, 2 Kbytes */
#define ADDR_FLASH_PAGE_64 (0x08020000UL)  /* Base address of Page 64, 2 Kbytes */
#define ADDR_FLASH_PAGE_65 (0x08020800UL)  /* Base address of Page 65, 2 Kbytes */
#define ADDR_FLASH_PAGE_66 (0x08021000UL)  /* Base address of Page 66, 2 Kbytes */
#define ADDR_FLASH_PAGE_67 (0x08021800UL)  /* Base address of Page 67, 2 Kbytes */
#define ADDR_FLASH_PAGE_68 (0x08022000UL)  /* Base address of Page 68, 2 Kbytes */
#define ADDR_FLASH_PAGE_69 (0x08022800UL)  /* Base address of Page 69, 2 Kbytes */
#define ADDR_FLASH_PAGE_70 (0x08023000UL)  /* Base address of Page 70, 2 Kbytes */
#define ADDR_FLASH_PAGE_71 (0x08023800UL)  /* Base address of Page 71, 2 Kbytes */
#define ADDR_FLASH_PAGE_72 (0x08024000UL)  /* Base address of Page 72, 2 Kbytes */
#define ADDR_FLASH_PAGE_73 (0x08024800UL)  /* Base address of Page 73, 2 Kbytes */
#define ADDR_FLASH_PAGE_74 (0x08025000UL)  /* Base address of Page 74, 2 Kbytes */
#define ADDR_FLASH_PAGE_75 (0x08025800UL)  /* Base address of Page 75, 2 Kbytes */
#define ADDR_FLASH_PAGE_76 (0x08026000UL)  /* Base address of Page 76, 2 Kbytes */
#define ADDR_FLASH_PAGE_77 (0x08026800UL)  /* Base address of Page 77, 2 Kbytes */
#define ADDR_FLASH_PAGE_78 (0x08027000UL)  /* Base address of Page 78, 2 Kbytes */
#define ADDR_FLASH_PAGE_79 (0x08027800UL)  /* Base address of Page 79, 2 Kbytes */
#define ADDR_FLASH_PAGE_80 (0x08028000UL)  /* Base address of Page 80, 2 Kbytes */
#define ADDR_FLASH_PAGE_81 (0x08028800UL)  /* Base address of Page 81, 2 Kbytes */
#define ADDR_FLASH_PAGE_82 (0x08029000UL)  /* Base address of Page 82, 2 Kbytes */
#define ADDR_FLASH_PAGE_83 (0x08029800UL)  /* Base address of Page 83, 2 Kbytes */
#define ADDR_FLASH_PAGE_84 (0x0802A000UL)  /* Base address of Page 84, 2 Kbytes */
#define ADDR_FLASH_PAGE_85 (0x0802A800UL)  /* Base address of Page 85, 2 Kbytes */
#define ADDR_FLASH_PAGE_86 (0x0802B000UL)  /* Base address of Page 86, 2 Kbytes */
#define ADDR_FLASH_PAGE_87 (0x0802B800UL)  /* Base address of Page 87, 2 Kbytes */
#define ADDR_FLASH_PAGE_88 (0x0802C000UL)  /* Base address of Page 88, 2 Kbytes */
#define ADDR_FLASH_PAGE_89 (0x0802C800UL)  /* Base address of Page 89, 2 Kbytes */
#define ADDR_FLASH_PAGE_90 (0x0802D000UL)  /* Base address of Page 90, 2 Kbytes */
#define ADDR_FLASH_PAGE_91 (0x0802D800UL)  /* Base address of Page 91, 2 Kbytes */
#define ADDR_FLASH_PAGE_92 (0x0802E000UL)  /* Base address of Page 92, 2 Kbytes */
#define ADDR_FLASH_PAGE_93 (0x0802E800UL)  /* Base address of Page 93, 2 Kbytes */
#define ADDR_FLASH_PAGE_94 (0x0802F000UL)  /* Base address of Page 94, 2 Kbytes */
#define ADDR_FLASH_PAGE_95 (0x0802F800UL)  /* Base address of Page 95, 2 Kbytes */
#define ADDR_FLASH_PAGE_96 (0x08030000UL)  /* Base address of Page 96, 2 Kbytes */
#define ADDR_FLASH_PAGE_97 (0x08030800UL)  /* Base address of Page 97, 2 Kbytes */
#define ADDR_FLASH_PAGE_98 (0x08031000UL)  /* Base address of Page 98, 2 Kbytes */
#define ADDR_FLASH_PAGE_99 (0x08031800UL)  /* Base address of Page 99, 2 Kbytes */
#define ADDR_FLASH_PAGE_100 (0x08032000UL) /* Base address of Page 100, 2 Kbytes */
#define ADDR_FLASH_PAGE_101 (0x08032800UL) /* Base address of Page 101, 2 Kbytes */
#define ADDR_FLASH_PAGE_102 (0x08033000UL) /* Base address of Page 102, 2 Kbytes */
#define ADDR_FLASH_PAGE_103 (0x08033800UL) /* Base address of Page 103, 2 Kbytes */
#define ADDR_FLASH_PAGE_104 (0x08034000UL) /* Base address of Page 104, 2 Kbytes */
#define ADDR_FLASH_PAGE_105 (0x08034800UL) /* Base address of Page 105, 2 Kbytes */
#define ADDR_FLASH_PAGE_106 (0x08035000UL) /* Base address of Page 106, 2 Kbytes */
#define ADDR_FLASH_PAGE_107 (0x08035800UL) /* Base address of Page 107, 2 Kbytes */
#define ADDR_FLASH_PAGE_108 (0x08036000UL) /* Base address of Page 108, 2 Kbytes */
#define ADDR_FLASH_PAGE_109 (0x08036800UL) /* Base address of Page 109, 2 Kbytes */
#define ADDR_FLASH_PAGE_110 (0x08037000UL) /* Base address of Page 110, 2 Kbytes */
#define ADDR_FLASH_PAGE_111 (0x08037800UL) /* Base address of Page 111, 2 Kbytes */
#define ADDR_FLASH_PAGE_112 (0x08038000UL) /* Base address of Page 112, 2 Kbytes */
#define ADDR_FLASH_PAGE_113 (0x08038800UL) /* Base address of Page 113, 2 Kbytes */
#define ADDR_FLASH_PAGE_114 (0x08039000UL) /* Base address of Page 114, 2 Kbytes */
#define ADDR_FLASH_PAGE_115 (0x08039800UL) /* Base address of Page 115, 2 Kbytes */
#define ADDR_FLASH_PAGE_116 (0x0803A000UL) /* Base address of Page 116, 2 Kbytes */
#define ADDR_FLASH_PAGE_117 (0x0803A800UL) /* Base address of Page 117, 2 Kbytes */
#define ADDR_FLASH_PAGE_118 (0x0803B000UL) /* Base address of Page 118, 2 Kbytes */
#define ADDR_FLASH_PAGE_119 (0x0803B800UL) /* Base address of Page 119, 2 Kbytes */
#define ADDR_FLASH_PAGE_120 (0x0803C000UL) /* Base address of Page 120, 2 Kbytes */
#define ADDR_FLASH_PAGE_121 (0x0803C800UL) /* Base address of Page 121, 2 Kbytes */
#define ADDR_FLASH_PAGE_122 (0x0803D000UL) /* Base address of Page 122, 2 Kbytes */
#define ADDR_FLASH_PAGE_123 (0x0803D800UL) /* Base address of Page 123, 2 Kbytes */
#define ADDR_FLASH_PAGE_124 (0x0803E000UL) /* Base address of Page 124, 2 Kbytes */
#define ADDR_FLASH_PAGE_125 (0x0803E800UL) /* Base address of Page 125, 2 Kbytes */
#define ADDR_FLASH_PAGE_126 (0x0803F000UL) /* Base address of Page 126, 2 Kbytes */
#define ADDR_FLASH_PAGE_127 (0x0803F800UL) /* Base address of Page 127, 2 Kbytes */

#ifdef __cplusplus
extern "C" {
#endif

static inline uint16_t STMFLASH_ReadHalfWord(uint32_t faddr) {
  return *(volatile uint16_t *)faddr;
}

static inline uint32_t STMFLASH_ReadWord(uint32_t faddr) {
  return *(volatile uint32_t *)faddr;
}

static inline uint64_t STMFLASH_ReadDoubleWord(uint32_t faddr) {
  return *(volatile uint64_t *)faddr;
}

int8_t STMFLASH_Write(uint32_t WriteAddr, uint64_t *pBuffer, uint32_t Num);
int8_t STMFLASH_Read(const uint32_t ReadAddr, uint64_t *pBuffer, uint32_t Num);
int8_t STMFLASH_Erase(uint32_t addrx, uint32_t length, uint32_t retry);

#ifdef __cplusplus
}
#endif

#endif
