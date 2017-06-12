/*
  Copyright 2017 by Frey Alfredsson <frea@itu.dk>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <regex.h>

#include <linux/types.h>

#define NVMED_IOCTL_QUEUE_DELETE _IOW('N', 0x52, unsigned int)

int main(void)
{
        char *proc_path = "/proc/nvmed/nvme0n1";
        char *admin_path = "/proc/nvmed/nvme0n1/admin";
        int fd;
        DIR *dir;
        struct dirent *ent = 0;
        regex_t regex;
        unsigned int qid;

        if (regcomp(&regex, "^[0-9]+$", REG_EXTENDED)) {
                fprintf(stderr, "Could not copmile regex\n");
                goto EXIT_REGEX;
        }


        if ((fd = open(admin_path, 0)) == 0) {
                fprintf(stderr, "Unable to open admin entry\n");
                goto EXIT_ADMIN;
        }

        printf("Cleaning up nvmed queues:\n");
        if ((dir = opendir(proc_path)) != NULL) {
                printf("  Closing queue(s): { ");
                while ((ent = readdir(dir)) != NULL) {
                        if (ent->d_type == DT_DIR && !regexec(&regex, ent->d_name, 0, NULL, 0)) {
                                qid = atoi(ent->d_name);
                                fflush(stdout);
                                ioctl(fd, NVMED_IOCTL_QUEUE_DELETE, &qid);
                                printf("%s ", ent->d_name);
                        }
                }
                printf("}\n");
        }
        closedir(dir);
        regfree(&regex);
        return EXIT_SUCCESS;
EXIT_ADMIN:
        regfree(&regex);
EXIT_REGEX:
        exit(EXIT_FAILURE);
}
