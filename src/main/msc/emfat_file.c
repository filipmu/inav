/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Author: jflyper@github.com
 */

#include "common/utils.h"
#include "common/printf.h"

#include "emfat.h"
#include "emfat_file.h"

#include "io/flashfs.h"

#define USE_EMFAT_AUTORUN
#define USE_EMFAT_ICON
//#define USE_EMFAT_README

#ifdef USE_EMFAT_AUTORUN
static const char autorun_file[] =
    "[autorun]\r\n"
    "icon=icon.ico\r\n"
    "label=iNav Onboard Flash\r\n" ;
#define AUTORUN_SIZE (sizeof(autorun_file) - 1)
#define EMFAT_INCR_AUTORUN 1
#else
#define EMFAT_INCR_AUTORUN 0
#endif

#ifdef USE_EMFAT_README
static const char readme_file[] =
    "This is readme file\r\n";
#define README_SIZE  (sizeof(readme_file) - 1)
#define EMFAT_INCR_README 1
#else
#define EMFAT_INCR_README 0
#endif

#ifdef USE_EMFAT_ICON
static const unsigned char icon_file[] =
{
  0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x18, 0x18, 0x00, 0x00, 0x01, 0x00,
  0x20, 0x00, 0x88, 0x09, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x3d, 0x0b,
  0x00, 0x00, 0x3d, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xd2, 0x97, 0x2a, 0x0b, 0xce, 0x91, 0x26, 0x52, 0xce, 0x90,
  0x25, 0x6e, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90,
  0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90,
  0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90,
  0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90,
  0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90,
  0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xce, 0x90,
  0x25, 0x6d, 0xce, 0x90, 0x25, 0x6d, 0xcf, 0x91, 0x26, 0x44, 0xd5, 0x9d,
  0x2e, 0x04, 0xcc, 0x8d, 0x23, 0x5d, 0xcb, 0x8a, 0x21, 0xf0, 0xcb, 0x8a,
  0x20, 0xfd, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a,
  0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a,
  0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a,
  0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a,
  0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a,
  0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfc, 0xcb, 0x8a,
  0x20, 0xfc, 0xcb, 0x8a, 0x20, 0xfd, 0xcb, 0x8b, 0x21, 0xe0, 0xcd, 0x8e,
  0x23, 0x37, 0xcb, 0x8b, 0x21, 0x81, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b,
  0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b,
  0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b,
  0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b,
  0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b,
  0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b,
  0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b,
  0x21, 0xff, 0xcb, 0x8b, 0x21, 0xff, 0xcb, 0x8b, 0x21, 0xf9, 0xcb, 0x8b,
  0x21, 0x53, 0xcd, 0x8d, 0x23, 0x81, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d,
  0x23, 0xff, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d,
  0x23, 0xff, 0xcc, 0x8d, 0x22, 0xff, 0xcc, 0x8c, 0x20, 0xff, 0xcc, 0x8d,
  0x21, 0xff, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d,
  0x23, 0xff, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d,
  0x23, 0xff, 0xcc, 0x8c, 0x21, 0xff, 0xcc, 0x8c, 0x20, 0xff, 0xcc, 0x8d,
  0x22, 0xff, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d,
  0x23, 0xff, 0xcc, 0x8d, 0x23, 0xff, 0xcc, 0x8d, 0x23, 0xf8, 0xcc, 0x8d,
  0x23, 0x53, 0xce, 0x8f, 0x24, 0x81, 0xce, 0x8f, 0x24, 0xff, 0xce, 0x8f,
  0x24, 0xff, 0xce, 0x8f, 0x24, 0xff, 0xce, 0x8f, 0x24, 0xff, 0xcd, 0x8e,
  0x23, 0xff, 0xcf, 0x94, 0x33, 0xff, 0xd3, 0xa0, 0x4c, 0xff, 0xd2, 0x9c,
  0x44, 0xff, 0xcd, 0x90, 0x28, 0xff, 0xcd, 0x8f, 0x23, 0xff, 0xce, 0x8f,
  0x24, 0xff, 0xce, 0x8f, 0x24, 0xff, 0xcd, 0x8f, 0x23, 0xff, 0xcd, 0x90,
  0x28, 0xff, 0xd2, 0x9c, 0x44, 0xff, 0xd3, 0xa0, 0x4c, 0xff, 0xcf, 0x94,
  0x33, 0xff, 0xcd, 0x8e, 0x23, 0xff, 0xce, 0x8f, 0x24, 0xff, 0xce, 0x8f,
  0x24, 0xff, 0xce, 0x8f, 0x24, 0xff, 0xce, 0x8f, 0x24, 0xf8, 0xcd, 0x8f,
  0x24, 0x53, 0xcf, 0x91, 0x26, 0x81, 0xcf, 0x91, 0x26, 0xff, 0xcf, 0x91,
  0x26, 0xff, 0xcf, 0x91, 0x26, 0xff, 0xce, 0x90, 0x24, 0xff, 0xd7, 0xa7,
  0x58, 0xff, 0xe7, 0xca, 0x9e, 0xff, 0xe4, 0xc4, 0x8d, 0xff, 0xe6, 0xc8,
  0x97, 0xff, 0xe2, 0xc1, 0x8b, 0xff, 0xd0, 0x97, 0x34, 0xff, 0xce, 0x91,
  0x25, 0xff, 0xce, 0x91, 0x25, 0xff, 0xd0, 0x97, 0x34, 0xff, 0xe2, 0xc1,
  0x8b, 0xff, 0xe6, 0xc8, 0x97, 0xff, 0xe4, 0xc4, 0x8d, 0xff, 0xe7, 0xca,
  0x9e, 0xff, 0xd7, 0xa7, 0x58, 0xff, 0xce, 0x90, 0x24, 0xff, 0xcf, 0x91,
  0x26, 0xff, 0xcf, 0x91, 0x26, 0xff, 0xcf, 0x91, 0x26, 0xf8, 0xcf, 0x91,
  0x26, 0x53, 0xd0, 0x93, 0x27, 0x81, 0xd0, 0x93, 0x27, 0xff, 0xd0, 0x93,
  0x27, 0xff, 0xcf, 0x93, 0x26, 0xff, 0xd4, 0x9e, 0x3f, 0xff, 0xe9, 0xcd,
  0xa0, 0xff, 0xd7, 0xa3, 0x47, 0xff, 0xcf, 0x93, 0x29, 0xff, 0xd0, 0x95,
  0x2b, 0xff, 0xe1, 0xbb, 0x78, 0xff, 0xe2, 0xbe, 0x82, 0xff, 0xcf, 0x93,
  0x27, 0xff, 0xcf, 0x93, 0x27, 0xff, 0xe2, 0xbe, 0x82, 0xff, 0xe1, 0xbb,
  0x78, 0xff, 0xd0, 0x95, 0x2b, 0xff, 0xcf, 0x93, 0x29, 0xff, 0xd7, 0xa3,
  0x47, 0xff, 0xe9, 0xcd, 0xa0, 0xff, 0xd4, 0x9e, 0x3f, 0xff, 0xcf, 0x92,
  0x26, 0xff, 0xd0, 0x93, 0x27, 0xff, 0xd0, 0x93, 0x27, 0xf8, 0xd0, 0x93,
  0x27, 0x53, 0xd1, 0x95, 0x29, 0x81, 0xd1, 0x95, 0x29, 0xff, 0xd1, 0x95,
  0x29, 0xff, 0xd0, 0x93, 0x25, 0xff, 0xde, 0xb3, 0x68, 0xff, 0xe2, 0xbd,
  0x7c, 0xff, 0xd2, 0x99, 0x32, 0xff, 0xea, 0xd1, 0xaa, 0xff, 0xde, 0xb6,
  0x70, 0xff, 0xd3, 0x9b, 0x35, 0xff, 0xe8, 0xcd, 0x9c, 0xff, 0xd3, 0x9a,
  0x33, 0xff, 0xd3, 0x9a, 0x33, 0xff, 0xe8, 0xcd, 0x9c, 0xff, 0xd3, 0x9b,
  0x35, 0xff, 0xde, 0xb6, 0x70, 0xff, 0xea, 0xd1, 0xaa, 0xff, 0xd2, 0x99,
  0x32, 0xff, 0xe2, 0xbd, 0x7c, 0xff, 0xde, 0xb3, 0x68, 0xff, 0xd0, 0x93,
  0x25, 0xff, 0xd1, 0x95, 0x29, 0xff, 0xd1, 0x95, 0x29, 0xf8, 0xd1, 0x95,
  0x29, 0x53, 0xd2, 0x97, 0x2a, 0x81, 0xd2, 0x97, 0x2a, 0xff, 0xd2, 0x97,
  0x2a, 0xff, 0xd1, 0x95, 0x27, 0xff, 0xdf, 0xb5, 0x69, 0xff, 0xe2, 0xbe,
  0x7d, 0xff, 0xd3, 0x9b, 0x33, 0xff, 0xee, 0xd7, 0xad, 0xff, 0xed, 0xd8,
  0xb4, 0xff, 0xd7, 0xa4, 0x4a, 0xff, 0xe7, 0xca, 0x94, 0xff, 0xd4, 0x9e,
  0x3c, 0xff, 0xd4, 0x9e, 0x3c, 0xff, 0xe7, 0xca, 0x94, 0xff, 0xd7, 0xa4,
  0x4a, 0xff, 0xed, 0xd8, 0xb4, 0xff, 0xee, 0xd7, 0xad, 0xff, 0xd3, 0x9b,
  0x33, 0xff, 0xe2, 0xbe, 0x7d, 0xff, 0xdf, 0xb5, 0x69, 0xff, 0xd1, 0x95,
  0x27, 0xff, 0xd2, 0x97, 0x2a, 0xff, 0xd2, 0x97, 0x2a, 0xf8, 0xd2, 0x97,
  0x2a, 0x53, 0xd3, 0x99, 0x2c, 0x81, 0xd3, 0x99, 0x2c, 0xff, 0xd3, 0x99,
  0x2c, 0xff, 0xd2, 0x98, 0x2a, 0xff, 0xd7, 0xa4, 0x43, 0xff, 0xea, 0xd0,
  0xa2, 0xff, 0xd7, 0xa4, 0x4a, 0xff, 0xd2, 0x98, 0x2d, 0xff, 0xdf, 0xb6,
  0x6a, 0xff, 0xe8, 0xcc, 0x9c, 0xff, 0xdb, 0xad, 0x5a, 0xff, 0xe7, 0xcb,
  0x9c, 0xff, 0xe7, 0xcb, 0x9c, 0xff, 0xdb, 0xad, 0x5a, 0xff, 0xe8, 0xcc,
  0x9c, 0xff, 0xdf, 0xb6, 0x6a, 0xff, 0xd2, 0x98, 0x2d, 0xff, 0xd7, 0xa5,
  0x4a, 0xff, 0xea, 0xd0, 0xa2, 0xff, 0xd7, 0xa4, 0x43, 0xff, 0xd2, 0x98,
  0x2a, 0xff, 0xd3, 0x99, 0x2c, 0xff, 0xd3, 0x99, 0x2c, 0xf8, 0xd3, 0x99,
  0x2c, 0x53, 0xd4, 0x9a, 0x2d, 0x81, 0xd4, 0x9b, 0x2d, 0xff, 0xd4, 0x9b,
  0x2d, 0xff, 0xd4, 0x9a, 0x2d, 0xff, 0xd3, 0x9a, 0x2c, 0xff, 0xde, 0xb2,
  0x5d, 0xff, 0xe9, 0xcf, 0xa1, 0xff, 0xe5, 0xc6, 0x93, 0xff, 0xe0, 0xba,
  0x77, 0xff, 0xdb, 0xac, 0x53, 0xff, 0xe2, 0xbe, 0x7d, 0xff, 0xfd, 0xfc,
  0xfa, 0xff, 0xfd, 0xfc, 0xfa, 0xff, 0xe2, 0xbe, 0x7d, 0xff, 0xdb, 0xac,
  0x53, 0xff, 0xe0, 0xba, 0x77, 0xff, 0xe5, 0xc6, 0x93, 0xff, 0xe9, 0xcf,
  0xa1, 0xff, 0xde, 0xb2, 0x5d, 0xff, 0xd3, 0x9a, 0x2c, 0xff, 0xd4, 0x9b,
  0x2d, 0xff, 0xd4, 0x9b, 0x2d, 0xff, 0xd4, 0x9b, 0x2d, 0xf8, 0xd4, 0x9b,
  0x2d, 0x53, 0xd5, 0x9c, 0x2e, 0x81, 0xd5, 0x9c, 0x2e, 0xff, 0xd5, 0x9c,
  0x2e, 0xff, 0xd5, 0x9c, 0x2e, 0xff, 0xd5, 0x9c, 0x2e, 0xff, 0xd4, 0x9b,
  0x2d, 0xff, 0xd8, 0xa3, 0x3c, 0xff, 0xdd, 0xaf, 0x55, 0xff, 0xda, 0xa9,
  0x49, 0xff, 0xd7, 0xa2, 0x3e, 0xff, 0xf3, 0xe5, 0xcd, 0xff, 0xf6, 0xea,
  0xd1, 0xff, 0xf6, 0xea, 0xd1, 0xff, 0xf3, 0xe5, 0xcd, 0xff, 0xd7, 0xa2,
  0x3e, 0xff, 0xda, 0xa9, 0x49, 0xff, 0xdd, 0xaf, 0x55, 0xff, 0xd8, 0xa3,
  0x3c, 0xff, 0xd4, 0x9b, 0x2d, 0xff, 0xd5, 0x9c, 0x2e, 0xff, 0xd5, 0x9c,
  0x2e, 0xff, 0xd5, 0x9c, 0x2e, 0xff, 0xd5, 0x9c, 0x2e, 0xf8, 0xd5, 0x9c,
  0x2e, 0x53, 0xd6, 0x9e, 0x2f, 0x81, 0xd6, 0x9e, 0x2f, 0xff, 0xd6, 0x9e,
  0x2f, 0xff, 0xd6, 0x9e, 0x2f, 0xff, 0xd6, 0x9e, 0x2f, 0xff, 0xd6, 0x9e,
  0x2f, 0xff, 0xd5, 0x9d, 0x2d, 0xff, 0xd4, 0x9b, 0x2a, 0xff, 0xd4, 0x9b,
  0x28, 0xff, 0xe0, 0xb6, 0x66, 0xff, 0xf2, 0xe1, 0xc1, 0xff, 0xd9, 0xa6,
  0x40, 0xff, 0xd9, 0xa6, 0x40, 0xff, 0xf2, 0xe1, 0xc1, 0xff, 0xe0, 0xb6,
  0x66, 0xff, 0xd4, 0x9b, 0x28, 0xff, 0xd4, 0x9b, 0x2a, 0xff, 0xd5, 0x9d,
  0x2d, 0xff, 0xd6, 0x9e, 0x2f, 0xff, 0xd6, 0x9e, 0x2f, 0xff, 0xd6, 0x9e,
  0x2f, 0xff, 0xd6, 0x9e, 0x2f, 0xff, 0xd6, 0x9e, 0x2f, 0xf8, 0xd6, 0x9e,
  0x2f, 0x53, 0xd7, 0xa0, 0x31, 0x81, 0xd7, 0xa0, 0x31, 0xff, 0xd7, 0xa0,
  0x31, 0xff, 0xd7, 0xa0, 0x31, 0xff, 0xd6, 0xa0, 0x30, 0xff, 0xd6, 0x9f,
  0x31, 0xff, 0xda, 0xac, 0x54, 0xff, 0xe0, 0xba, 0x75, 0xff, 0xdc, 0xb1,
  0x5f, 0xff, 0xe1, 0xb9, 0x69, 0xff, 0xf0, 0xdd, 0xba, 0xff, 0xd6, 0xa1,
  0x37, 0xff, 0xd6, 0xa1, 0x37, 0xff, 0xf0, 0xdd, 0xba, 0xff, 0xe1, 0xb9,
  0x69, 0xff, 0xdc, 0xb1, 0x5f, 0xff, 0xe0, 0xba, 0x75, 0xff, 0xda, 0xac,
  0x54, 0xff, 0xd6, 0x9f, 0x31, 0xff, 0xd6, 0xa0, 0x30, 0xff, 0xd6, 0xa0,
  0x31, 0xff, 0xd6, 0x9f, 0x31, 0xff, 0xd7, 0xa0, 0x31, 0xf8, 0xd7, 0xa0,
  0x31, 0x53, 0xd7, 0xa1, 0x32, 0x81, 0xd7, 0xa1, 0x32, 0xff, 0xd7, 0xa1,
  0x32, 0xff, 0xd7, 0xa1, 0x32, 0xff, 0xd7, 0xa1, 0x32, 0xff, 0xe2, 0xbe,
  0x79, 0xff, 0xeb, 0xd1, 0xa0, 0xff, 0xe5, 0xc2, 0x79, 0xff, 0xe1, 0xb9,
  0x6a, 0xff, 0xe2, 0xbf, 0x7c, 0xff, 0xf0, 0xdd, 0xba, 0xff, 0xed, 0xd8,
  0xb3, 0xff, 0xed, 0xd8, 0xb3, 0xff, 0xf0, 0xdd, 0xba, 0xff, 0xe2, 0xbf,
  0x7c, 0xff, 0xe1, 0xb9, 0x6a, 0xff, 0xe5, 0xc1, 0x79, 0xff, 0xeb, 0xd1,
  0xa0, 0xff, 0xe2, 0xbe, 0x79, 0xff, 0xd7, 0xa1, 0x32, 0xff, 0xd7, 0xa1,
  0x32, 0xff, 0xd7, 0xa1, 0x32, 0xff, 0xd7, 0xa1, 0x32, 0xf8, 0xd7, 0xa1,
  0x32, 0x53, 0xd8, 0xa2, 0x33, 0x81, 0xd8, 0xa2, 0x33, 0xff, 0xd8, 0xa2,
  0x33, 0xff, 0xd8, 0xa2, 0x31, 0xff, 0xdd, 0xb0, 0x53, 0xff, 0xec, 0xd3,
  0xa1, 0xff, 0xda, 0xa8, 0x3f, 0xff, 0xd8, 0xa5, 0x40, 0xff, 0xe5, 0xc4,
  0x87, 0xff, 0xe9, 0xcd, 0x94, 0xff, 0xe3, 0xbf, 0x78, 0xff, 0xe4, 0xc0,
  0x75, 0xff, 0xe4, 0xc0, 0x75, 0xff, 0xe3, 0xbe, 0x78, 0xff, 0xe9, 0xcd,
  0x94, 0xff, 0xe5, 0xc4, 0x87, 0xff, 0xd8, 0xa5, 0x40, 0xff, 0xda, 0xa8,
  0x3f, 0xff, 0xec, 0xd3, 0xa1, 0xff, 0xdd, 0xb0, 0x53, 0xff, 0xd8, 0xa2,
  0x31, 0xff, 0xd8, 0xa2, 0x33, 0xff, 0xd8, 0xa2, 0x33, 0xf8, 0xd8, 0xa2,
  0x33, 0x53, 0xd9, 0xa4, 0x34, 0x81, 0xd9, 0xa4, 0x34, 0xff, 0xd9, 0xa4,
  0x34, 0xff, 0xd8, 0xa2, 0x30, 0xff, 0xe4, 0xbf, 0x73, 0xff, 0xe6, 0xc4,
  0x7d, 0xff, 0xda, 0xa9, 0x40, 0xff, 0xf3, 0xe5, 0xcc, 0xff, 0xf0, 0xdc,
  0xb3, 0xff, 0xdc, 0xab, 0x45, 0xff, 0xeb, 0xd2, 0x9e, 0xff, 0xda, 0xa8,
  0x3d, 0xff, 0xda, 0xa8, 0x3d, 0xff, 0xeb, 0xd2, 0x9e, 0xff, 0xdc, 0xab,
  0x45, 0xff, 0xf0, 0xdc, 0xb3, 0xff, 0xf3, 0xe5, 0xcc, 0xff, 0xda, 0xa9,
  0x40, 0xff, 0xe6, 0xc4, 0x7d, 0xff, 0xe4, 0xc0, 0x73, 0xff, 0xd8, 0xa2,
  0x30, 0xff, 0xd9, 0xa4, 0x34, 0xff, 0xd9, 0xa4, 0x34, 0xf8, 0xd9, 0xa4,
  0x34, 0x53, 0xd9, 0xa5, 0x35, 0x81, 0xd9, 0xa5, 0x35, 0xff, 0xd9, 0xa5,
  0x35, 0xff, 0xd9, 0xa4, 0x32, 0xff, 0xe3, 0xbc, 0x6a, 0xff, 0xe8, 0xc9,
  0x8b, 0xff, 0xda, 0xa7, 0x39, 0xff, 0xe9, 0xcb, 0x8a, 0xff, 0xe2, 0xb9,
  0x60, 0xff, 0xdc, 0xac, 0x47, 0xff, 0xec, 0xd3, 0xa1, 0xff, 0xdb, 0xa9,
  0x3c, 0xff, 0xdb, 0xa8, 0x3d, 0xff, 0xec, 0xd3, 0xa1, 0xff, 0xdc, 0xac,
  0x47, 0xff, 0xe2, 0xb9, 0x60, 0xff, 0xe9, 0xcb, 0x8a, 0xff, 0xda, 0xa7,
  0x39, 0xff, 0xe8, 0xc9, 0x8c, 0xff, 0xe3, 0xbc, 0x6a, 0xff, 0xd9, 0xa4,
  0x32, 0xff, 0xd9, 0xa5, 0x35, 0xff, 0xd9, 0xa5, 0x35, 0xf8, 0xd9, 0xa5,
  0x35, 0x53, 0xda, 0xa6, 0x35, 0x81, 0xda, 0xa6, 0x36, 0xff, 0xda, 0xa6,
  0x35, 0xff, 0xda, 0xa6, 0x35, 0xff, 0xdc, 0xac, 0x42, 0xff, 0xec, 0xd4,
  0xa2, 0xff, 0xe1, 0xb9, 0x6b, 0xff, 0xd9, 0xa6, 0x3c, 0xff, 0xdb, 0xaa,
  0x47, 0xff, 0xe9, 0xcc, 0x95, 0xff, 0xe6, 0xc4, 0x7a, 0xff, 0xda, 0xa5,
  0x34, 0xff, 0xda, 0xa5, 0x34, 0xff, 0xe6, 0xc4, 0x79, 0xff, 0xe9, 0xcc,
  0x95, 0xff, 0xdb, 0xaa, 0x47, 0xff, 0xd9, 0xa6, 0x3c, 0xff, 0xe1, 0xb9,
  0x6b, 0xff, 0xec, 0xd4, 0xa2, 0xff, 0xdc, 0xac, 0x42, 0xff, 0xda, 0xa6,
  0x35, 0xff, 0xda, 0xa6, 0x35, 0xff, 0xda, 0xa6, 0x36, 0xf8, 0xda, 0xa6,
  0x35, 0x53, 0xdb, 0xa7, 0x36, 0x81, 0xdb, 0xa7, 0x36, 0xff, 0xdb, 0xa7,
  0x36, 0xff, 0xdb, 0xa7, 0x36, 0xff, 0xda, 0xa7, 0x35, 0xff, 0xdf, 0xb2,
  0x50, 0xff, 0xeb, 0xd0, 0x97, 0xff, 0xeb, 0xd2, 0xa1, 0xff, 0xec, 0xd3,
  0xa2, 0xff, 0xe7, 0xc5, 0x7b, 0xff, 0xdb, 0xa9, 0x3a, 0xff, 0xdb, 0xa7,
  0x36, 0xff, 0xdb, 0xa7, 0x36, 0xff, 0xdb, 0xa9, 0x3a, 0xff, 0xe7, 0xc5,
  0x7b, 0xff, 0xec, 0xd3, 0xa2, 0xff, 0xeb, 0xd2, 0xa1, 0xff, 0xeb, 0xd0,
  0x97, 0xff, 0xdf, 0xb2, 0x50, 0xff, 0xda, 0xa7, 0x35, 0xff, 0xdb, 0xa7,
  0x36, 0xff, 0xdb, 0xa7, 0x36, 0xff, 0xdb, 0xa7, 0x36, 0xf8, 0xdb, 0xa7,
  0x36, 0x53, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa7,
  0x36, 0xff, 0xdb, 0xa9, 0x39, 0xff, 0xde, 0xae, 0x44, 0xff, 0xdd, 0xac,
  0x40, 0xff, 0xdb, 0xa7, 0x35, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa7,
  0x35, 0xff, 0xdd, 0xac, 0x40, 0xff, 0xde, 0xae, 0x44, 0xff, 0xdb, 0xa9,
  0x39, 0xff, 0xdb, 0xa7, 0x36, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xf8, 0xdb, 0xa8,
  0x37, 0x53, 0xdb, 0xa8, 0x37, 0x82, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x36, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x36, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xf8, 0xdb, 0xa8,
  0x37, 0x54, 0xdb, 0xa8, 0x37, 0x63, 0xdb, 0xa8, 0x37, 0xf6, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8,
  0x37, 0xff, 0xdb, 0xa8, 0x37, 0xff, 0xdb, 0xa8, 0x37, 0xe7, 0xdb, 0xa8,
  0x37, 0x3c, 0xdb, 0xa8, 0x37, 0x10, 0xdb, 0xa8, 0x37, 0x63, 0xdb, 0xa8,
  0x37, 0x82, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8,
  0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8,
  0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8,
  0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8,
  0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8,
  0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8,
  0x37, 0x81, 0xdb, 0xa8, 0x37, 0x81, 0xdb, 0xa8, 0x37, 0x53, 0xdb, 0xa8,
  0x37, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00
};

#define ICON_SIZE    (sizeof(icon_file))
#define EMFAT_INCR_ICON 1
#else
#define EMFAT_INCR_ICON 0
#endif

#define CMA_TIME EMFAT_ENCODE_CMA_TIME(1,1,2018, 13,0,0)
#define CMA { CMA_TIME, CMA_TIME, CMA_TIME }

static void memory_read_proc(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry)
{
    int len;

    if (offset > entry->curr_size) {
        return;
    }

    if (offset + size > entry->curr_size) {
        len = entry->curr_size - offset;
    } else {
        len = size;
    }

    memcpy(dest, &((char *)entry->user_data)[offset], len);
}

static void bblog_read_proc(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry)
{
    UNUSED(entry);

    flashfsReadAbs(offset, dest, size);
}

static const emfat_entry_t entriesPredefined[] =
{
    // name           dir    attr         lvl offset  size             max_size        user                time  read               write
    { "",             true,  0,           0,  0,      0,               0,              0,                  CMA,  NULL,              NULL, { 0 } },
#ifdef USE_EMFAT_AUTORUN
    { "autorun.inf",  false, ATTR_HIDDEN, 1,  0,      AUTORUN_SIZE,    AUTORUN_SIZE,   (long)autorun_file, CMA,  memory_read_proc,  NULL, { 0 } },
#endif
#ifdef USE_EMFAT_ICON
    { "icon.ico",     false, ATTR_HIDDEN, 1,  0,      ICON_SIZE,       ICON_SIZE,      (long)icon_file,    CMA,  memory_read_proc,  NULL, { 0 } },
#endif
#ifdef USE_EMFAT_README
    { "readme.txt",   false, 0,           1,  0,      README_SIZE,     1024*1024,      (long)readme_file,  CMA,  memory_read_proc,  NULL, { 0 } },
#endif
    { "INAV_LOG.BBL", 0,     0,           1,  0,      0,               0,              0,                  CMA,  bblog_read_proc,   NULL, { 0 } },
};

#define ENTRY_INDEX_BBL (1 + EMFAT_INCR_AUTORUN + EMFAT_INCR_ICON + EMFAT_INCR_README)

#define EMFAT_MAX_LOG_ENTRY 100
#define EMFAT_MAX_ENTRY (ENTRY_INDEX_BBL + EMFAT_MAX_LOG_ENTRY)

static emfat_entry_t entries[EMFAT_MAX_ENTRY];
static char logNames[EMFAT_MAX_LOG_ENTRY][8+3];

emfat_t emfat;

static void emfat_add_log(emfat_entry_t *entry, int number, uint32_t offset, uint32_t size)
{
    tfp_sprintf(logNames[number], "INAV_%03d.BBL", number);
    entry->name = logNames[number];
    entry->level = 1;
    entry->offset = offset;
    entry->curr_size = size;
    entry->max_size = entry->curr_size;
    entry->cma_time[0] = CMA_TIME;
    entry->cma_time[1] = CMA_TIME;
    entry->cma_time[2] = CMA_TIME;
    entry->readcb = bblog_read_proc;
}

static void emfat_find_log(emfat_entry_t *entry, int maxCount)
{
    uint32_t limit  = flashfsIdentifyStartOfFreeSpace();
    uint32_t lastOffset = 0;
    uint32_t currOffset = 0;
    int fileNumber = 0;
    uint8_t buffer[18];

    for ( ; currOffset < limit ; currOffset += 2048) { // XXX 2048 = FREE_BLOCK_SIZE in io/flashfs.c

        flashfsReadAbs(currOffset, buffer, 18);

        if (strncmp((char *)buffer, "H Product:Blackbox", 18)) {
            continue;
        }

        if (lastOffset != currOffset) {
            emfat_add_log(entry, fileNumber, lastOffset, currOffset - lastOffset);

            ++fileNumber;
            if (fileNumber == maxCount) {
                break;
            }
            ++entry;
        }

        lastOffset = currOffset;
    }

    if (fileNumber != maxCount && lastOffset != currOffset) {
        emfat_add_log(entry, fileNumber, lastOffset, currOffset - lastOffset);
    }
}

void emfat_init_files(void)
{
    memset(entries, 0, sizeof(entries));

    for (size_t i = 0 ; i < ARRAYLEN(entriesPredefined) ; i++) {
        entries[i] = entriesPredefined[i];
    }

    // Singleton
    emfat_entry_t *entry = &entries[ENTRY_INDEX_BBL];
    entry->curr_size = flashfsIdentifyStartOfFreeSpace();
    entry->max_size = flashfsGetSize();

    // Detect and list individual power cycle sessions
    emfat_find_log(&entries[ENTRY_INDEX_BBL + 1], EMFAT_MAX_ENTRY - (ENTRY_INDEX_BBL + 1));

    emfat_init(&emfat, "emfat", entries);
}
