/* Image capture */

#define IMG_FMT_RAW 1
#define IMG_FMT_BMP 2
#define IMG_FMT_JPG 3
#define FILE_NAME_SIZE 50

struct image_data {
        unsigned char *image_buf;
        int width;
        int height;
        int cam_num;
        int captr_end;
} st_image_data[MAX_CAM];

typedef uint8_t  BYTE;
int image_capture(int num_cam, guint8 ** cap_ptr, unsigned char *fullbuffer,
                int save_image_len, struct cmdline_args cmdline,
                struct app_data *app_data);
int save_jpg(const char *Filename, int imgsize, BYTE * data);
int jpg_encoding(unsigned char *image_data, int width, int height, int num, int captr_end);
int yuv_rgb888_conversion(unsigned char *yuyv_buffer, unsigned char *rgb_buffer, int width_act, int height_act);
void save_bmp_image(unsigned char *fullbuffer, int width, int height, unsigned long int size, int num, int captr_end);
void save_raw_image(unsigned char *fullbuffer, int width, int height, unsigned long int size, int num, int captr_end);
int write_jpeg_file(unsigned char *image_data, int width, int height, int num, int captr_end , int image_size );

int write_image(struct image_data *image);
/* ***************************************************************************** */
