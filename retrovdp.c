
#include <maug.h>
#include <retroflt.h>

#ifndef RETROFLAT_API_SDL1
#   error "NTSC presently only works with SDL 1!"
#endif /* !RETROFLAT_API_SDL1 */

#define NTSC_C
#include "ntsc.h"

struct VDP_DATA {
   struct NTSC_SETTINGS ntsc;
   struct CRT crt;
   int field;
   int noise;
};

MERROR_RETVAL retroflat_vdp_init( struct RETROFLAT_STATE* state ) {
   MERROR_RETVAL retval = MERROR_OK;
   struct VDP_DATA* data = NULL;

   debug_printf( 1, "setting up NTSC..." );

   state->vdp_buffer = calloc( 1, sizeof( struct RETROFLAT_BITMAP ) );
   maug_cleanup_if_null_alloc( struct RETROFLAT_BITMAP*, state->vdp_buffer );
   state->vdp_data = calloc( 1, sizeof( struct VDP_DATA ) );
   maug_cleanup_if_null_alloc( void*, state->vdp_data );
   data = (struct VDP_DATA*)(state->vdp_data);

   /* Create intermediary screen buffer. */
   retval = state->vdp_create_bitmap(
      state->screen_v_w, state->screen_v_h, state->vdp_buffer,
      RETROFLAT_FLAGS_OPAQUE );

   /* Initialize CRT buffer. */
   crt_init(
      &(data->crt), state->screen_v_w, state->screen_v_h,
      CRT_PIX_FORMAT_RGBA, state->buffer.surface->pixels );
   data->crt.blend = 1;
   data->crt.scanlines = 1;

   data->ntsc.w = state->screen_v_w;
   data->ntsc.h = state->screen_v_h;
   data->ntsc.as_color = 1;
   data->ntsc.raw = 0;
   if( 0 < strlen( state->vdp_args ) ) {
      debug_printf( 1, "NTSC noise: %d", atoi( state->vdp_args ) );
      data->noise = atoi( state->vdp_args );
   }

cleanup:

   return retval;
}

void retroflat_vdp_shutdown( struct RETROFLAT_STATE* state ) {
   state->vdp_destroy_bitmap( state->vdp_buffer );
   free( state->vdp_buffer );
   free( state->vdp_data );
}

MERROR_RETVAL retroflat_vdp_flip( struct RETROFLAT_STATE* state ) {
   MERROR_RETVAL retval = MERROR_OK;
   struct VDP_DATA* data = (struct VDP_DATA*)(state->vdp_data);

   assert(
      4 == g_retroflat_state->buffer.surface->format->BytesPerPixel );

   retroflat_px_lock( &(state->buffer) );
   retroflat_px_lock( state->vdp_buffer );

   data->ntsc.data = state->vdp_buffer->surface->pixels;
   data->crt.out = state->buffer.surface->pixels;

   data->ntsc.format = CRT_PIX_FORMAT_RGBA;
   data->ntsc.field = data->field & 1;
   if( 0 == data->ntsc.field ) {
      data->ntsc.frame ^= 1;
   }
   crt_modulate( &(data->crt), &(data->ntsc) );
   crt_demodulate( &(data->crt), data->noise );
   data->ntsc.field ^= 1;

   retroflat_px_release( &(state->buffer) );
   retroflat_px_release( state->vdp_buffer );

   return retval;
}

