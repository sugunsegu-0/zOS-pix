gboolean bus_message(GstBus * bus, GstMessage * message, struct gst_handler *gst_handle);
int get_display_resolution(struct cmdline_args *cmdline);
int preview_frames_on_display(guint8 ** cap_ptr, struct gst_handler *gst_handle[],
		unsigned char *framebuffer, int cameras_connected, int buffer_size);

int gst_push_buffer(guint8 ** cap_ptr, struct gst_handler *gst_handle,
		unsigned char *framebuffer, int cameras_connected,
					int camera_num, int buffer_size);
void calcAspect(int *width, int *height, int *offset, int cameras_connected);
int create_gst_handle(struct gst_handler *gst_handle, short int cameras_connected, int is_background);
int start_stream(struct gst_handler *gst_handle);
int stream_change_state(struct gst_handler *gst_handle, int state);
void free_gst_handle(struct gst_handler *gst_handle);
void fill_display_background(void *dest, unsigned int size);

