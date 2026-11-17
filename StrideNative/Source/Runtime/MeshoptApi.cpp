// glTF meshopt decoding; batches decode buffer views in parallel.
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Runtime.h>

#include <meshoptimizer.h>

namespace
{
	int32_t Decode(void *inDst, uint32_t inCount, uint32_t inStride, const void *inSrc, uint32_t inSrcSize, int32_t inMode, int32_t inFilter)
	{
		if (inDst == nullptr || inSrc == nullptr || inStride == 0)
			return SN_ERROR_INVALID_ARGUMENT;

		const unsigned char *src = static_cast<const unsigned char *>(inSrc);
		int result;
		switch (inMode)
		{
		case SN_MESHOPT_MODE_ATTRIBUTES:	result = meshopt_decodeVertexBuffer(inDst, inCount, inStride, src, inSrcSize); break;
		case SN_MESHOPT_MODE_TRIANGLES:		result = meshopt_decodeIndexBuffer(inDst, inCount, inStride, src, inSrcSize); break;
		case SN_MESHOPT_MODE_INDICES:		result = meshopt_decodeIndexSequence(inDst, inCount, inStride, src, inSrcSize); break;
		default:							return SN_ERROR_INVALID_ARGUMENT;
		}
		if (result != 0)
			return SN_ERROR_FAILED;

		switch (inFilter)
		{
		case SN_MESHOPT_FILTER_NONE:		break;
		case SN_MESHOPT_FILTER_OCTAHEDRAL:	meshopt_decodeFilterOct(inDst, inCount, inStride); break;
		case SN_MESHOPT_FILTER_QUATERNION:	meshopt_decodeFilterQuat(inDst, inCount, inStride); break;
		case SN_MESHOPT_FILTER_EXPONENTIAL:	meshopt_decodeFilterExp(inDst, inCount, inStride); break;
		case SN_MESHOPT_FILTER_COLOR:		meshopt_decodeFilterColor(inDst, inCount, inStride); break;
		default:							return SN_ERROR_INVALID_ARGUMENT;
		}
		return SN_OK;
	}
}

using namespace StrideNative;

SN_API int32_t sn_meshopt_decode(void *dst, uint32_t count, uint32_t stride, const void *src, uint32_t src_size, int32_t mode, int32_t filter)
{
	int32_t result = Decode(dst, count, stride, src, src_size, mode, filter);
	if (result != SN_OK)
		SetLastError(result, "meshopt decode failed (invalid arguments or corrupt data)");
	return result;
}

SN_API int32_t sn_meshopt_decode_batch(sn_meshopt_request *requests, uint32_t count)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (requests == nullptr && count > 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "requests is null");

	return RunParallelTask("MeshoptDecode", count, [requests](Task &ioTask, uint32_t inIndex)
	{
		sn_meshopt_request &request = requests[inIndex];
		request.result = Decode(request.dst, request.count, request.stride, request.src, request.src_size, request.mode, request.filter);
		if (request.result != SN_OK)
			ioTask.Fail("meshopt decode failed for request " + std::to_string(inIndex));
	}, {});
}
