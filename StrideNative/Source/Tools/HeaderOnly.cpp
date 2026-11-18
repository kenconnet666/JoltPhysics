// Implementations of the header-only libraries, compiled once for the tools module.
// Their include directories are SYSTEM, so their warnings do not hit our -Werror.
#define CGLTF_IMPLEMENTATION
#include <cgltf.h>

#define STB_TRUETYPE_IMPLEMENTATION
#include <stb_truetype.h>

#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#define STBI_ONLY_TGA
#define STBI_NO_STDIO
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define ENABLE_VHACD_IMPLEMENTATION 1
#include <VHACD.h>
