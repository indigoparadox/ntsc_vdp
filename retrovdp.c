
#define RETROVDP_C
#include <maug.h>
#include <retroflt.h>

#if !defined( RETROFLAT_API_SDL1 ) && !defined( RETROFLAT_API_WIN32 )
#   error "NTSC presently only works with SDL1 or WIN32!"
#endif /* !RETROFLAT_API_SDL1 && !RETROFLAT_API_WIN32 */

#define NTSC_C
#include "ntsc.h"

struct VDP_DATA {
   struct NTSC_SETTINGS ntsc;
   struct CRT crt;
   int field;
   int noise;
};

#ifdef RETROFLAT_OS_WIN
extern __declspec( dllexport )
#endif /* RETROFLAT_OS_WIN */
MERROR_RETVAL retroflat_vdp_init( struct RETROFLAT_STATE* state ) {
   MERROR_RETVAL retval = MERROR_OK;
   struct VDP_DATA* data = NULL;

   printf( "setting up NTSC...\n" );

   state->vdp_data = calloc( 1, sizeof( struct VDP_DATA ) );
   maug_cleanup_if_null_alloc( void*, state->vdp_data );
   data = (struct VDP_DATA*)(state->vdp_data);

   state->vdp_flags = RETROFLAT_VDP_FLAG_PXLOCK;

   /* Create intermediary screen buffer. */
   debug_printf( 1, "creating VDP buffer, %d x %d",
      state->screen_v_w, state->screen_v_h );
   state->vdp_buffer = calloc( 1, sizeof( struct RETROFLAT_BITMAP ) );
   maug_cleanup_if_null_alloc( struct RETROFLAT_BITMAP*, state->vdp_buffer );
   retval = state->vdp_create_bitmap(
      state->screen_v_w, state->screen_v_h, state->vdp_buffer,
      RETROFLAT_FLAGS_OPAQUE );

   /* Initialize CRT buffer. */
   crt_init(
      &(data->crt), state->screen_v_w, state->screen_v_h,
#ifdef RETROFLAT_API_SDL1
      CRT_PIX_FORMAT_RGBA, state->buffer.surface->pixels );
#else
      /* TODO */
      CRT_PIX_FORMAT_BGRA, NULL );
#endif /* RETROFLAT_API_SDL1 */
   data->crt.blend = 1;
   data->crt.scanlines = 1;

   data->ntsc.w = state->screen_v_w;
   data->ntsc.h = state->screen_v_h;
   data->ntsc.as_color = 1;
   data->ntsc.raw = 0;
   if( 0 < strlen( state->vdp_args ) ) {
      printf( "NTSC noise: %d\n", atoi( state->vdp_args ) );
      data->noise = atoi( state->vdp_args );
   }

cleanup:

   return retval;
}

#ifdef RETROFLAT_OS_WIN
extern __declspec( dllexport )
#endif /* RETROFLAT_OS_WIN */
void retroflat_vdp_shutdown( struct RETROFLAT_STATE* state ) {
   printf( "shutting down NTSC...\n" );
   state->vdp_destroy_bitmap( state->vdp_buffer );
   free( state->vdp_buffer );
   free( state->vdp_data );
}

#ifdef RETROFLAT_OS_WIN
extern __declspec( dllexport )
#endif /* RETROFLAT_OS_WIN */
MERROR_RETVAL retroflat_vdp_flip( struct RETROFLAT_STATE* state ) {
   MERROR_RETVAL retval = MERROR_OK;
   struct VDP_DATA* data = (struct VDP_DATA*)(state->vdp_data);

#ifdef RETROFLAT_API_SDL1
   assert( 4 == state->buffer.surface->format->BytesPerPixel );
#endif /* RETROFLAT_API_SDL1 */
   assert( 0 < state->screen_v_w );
   assert( 0 < state->screen_v_h );
   assert( NULL != data );
   assert( NULL != state->vdp_buffer );

#ifdef RETROFLAT_API_SDL1
   data->ntsc.data = state->vdp_buffer->surface->pixels;
   data->crt.out = state->buffer.surface->pixels;
   data->ntsc.format = CRT_PIX_FORMAT_RGBA;
#else
   assert( NULL != state->vdp_buffer->bits );

   data->ntsc.data = state->vdp_buffer->bits;
   data->crt.out = state->buffer.bits;
   data->ntsc.format = CRT_PIX_FORMAT_BGRA;
#endif /* RETROFLAT_API_SDL1 */

   assert( NULL != data->ntsc.data );
   assert( NULL != data->crt.out );

   data->ntsc.field = data->field & 1;
   if( 0 == data->ntsc.field ) {
      data->ntsc.frame ^= 1;
   }
   crt_modulate( &(data->crt), &(data->ntsc) );
   crt_demodulate( &(data->crt), data->noise );
   data->ntsc.field ^= 1;

   return retval;
}

