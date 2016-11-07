/*
 * flash_fs.h
 *
 *  Created on: 24.08.2016
 *      Author: Sebastian Förster
 *      Website: sebstianfoerster86.wordpress.com
 *      License: GPLv2
 */

#ifndef FLASH_FS_H_
#define FLASH_FS_H_

#include <stdio.h>
#include <string.h>

extern int my_heap_cnt;

#define FLASH_FS_USE

extern void flash_fs_init(void);
extern int flash_seek_file(int file_desc, int ptr, int dir);
extern int flash_read_text_file(int file_desc, char *buf, int len);
extern int flash_write(int file_desc, char *buf, int len);
extern int flash_close_file(int file_desc);
extern int flash_open_file(const char *file_name);

typedef struct falsh_file_s
{
	const unsigned char 	*file;
	size_t 					size;
	char 					file_num;
	int 					file_pointer;
} flash_file;

#endif /* FLASH_FS_H_ */
