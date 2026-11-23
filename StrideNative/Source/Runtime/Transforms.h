// 变换存储：TransformApi.cpp 实现 C API，物理也直接读写它（绑定了槽位的物体）。
#pragma once

#include <Core/Core.h>

#include <Jolt/Math/Mat44.h>
#include <Jolt/Math/DMat44.h>

#include <vector>

namespace StrideNative
{
	/// 结构数组布局的变换。槽位从 1 开始，0 是空槽位（parent 为 0 表示根），各列都有 capacity + 1 项。
	/// 列在创建时一次分配好，指针不变，JS 可以一直保留视图。
	///
	/// 大世界（D18）：本地位置和世界矩阵是双精度（几十公里外也不抖）；另有一份单精度的"渲染用世界矩阵"，平移是相对渲染原点
	/// （相机附近、按格对齐，见 SetRenderOrigin）的，GPU 和剔除用它。旋转、缩放仍是单精度。
	class TransformStore
	{
	public:
		explicit TransformStore(uint32_t inCapacity);

		uint32_t Capacity() const { return mCapacity; }
		uint32_t SlotLimit() const { return mHighWater; }
		bool IsAlive(int32_t inSlot) const { return inSlot > 0 && uint32_t(inSlot) <= mCapacity && mAlive[inSlot] != 0; }

		/// 分配 inCount 个槽位；容量不够时什么都不做，返回 false。
		bool Allocate(int32_t *outSlots, uint32_t inCount);
		/// 释放槽位；有留下的子节点时什么都不做，在 outError 里说明原因。
		bool Free(const int32_t *inSlots, uint32_t inCount, std::string &outError);
		bool SetParent(int32_t inSlot, int32_t inParent, std::string &outError);
		/// 链接（Stride 的 TransformLink）：world = inLinkSlot 的 world * inMatrix * 本地，不看父节点；inLinkSlot 为 0 时去掉链接
		bool SetLink(int32_t inSlot, int32_t inLinkSlot, const float *inMatrix, std::string &outError);

		/// 按父节点在前的顺序算所有世界矩阵。
		void Update();
		/// 只算 inSlot 和它的祖先的世界矩阵，从根往下写
		void UpdateSlot(int32_t inSlot);

		/// 上一次 Update 时世界矩阵变了的槽位（和再上一次 Update 比；那之间分配、释放、UpdateSlot 改过的也算，不重复）。
		const std::vector<int32_t> &Changed() const { return mPublished; }

		/// 本地矩阵 T * R * S。
		JPH::DMat44 LocalMatrix(int32_t inSlot) const;
		/// 沿父链现算世界矩阵，不依赖上一次 Update（加物体时用：槽位可能刚设好位姿）。
		JPH::DMat44 ComputeWorldMatrix(int32_t inSlot) const;
		/// 把世界空间的位姿写成本地位置和旋转；有父节点时用父节点上一次 Update 的世界矩阵换算。
		void SetWorldPose(int32_t inSlot, JPH::DVec3Arg inPosition, JPH::QuatArg inRotation);
		/// 渲染原点：渲染用世界矩阵的平移减去它。变了时所有活着的槽位的渲染用矩阵重算、都记成变了（GPU 上的要全部重传一次），
		/// 所以调用方按格对齐、只在相机走出一格时才换。
		void SetRenderOrigin(JPH::DVec3Arg inOrigin);
		JPH::DVec3 RenderOrigin() const { return mRenderOrigin; }

		std::vector<double>		mPosition;			///< xyz
		std::vector<float>		mRotation;			///< xyzw
		std::vector<float>		mScale;				///< xyz
		std::vector<int32_t>	mParent;			///< 0 表示根
		std::vector<double>		mWorld;				///< 列主序 4x4
		std::vector<float>		mRenderWorld;		///< 列主序 4x4，平移相对渲染原点

	private:
		JPH::DMat44				LoadWorld(int32_t inSlot) const;
		/// 算世界矩阵时的"父"：有链接时是链接的槽位，否则是父节点
		int32_t					EffectiveParent(int32_t inSlot) const { return mLinkSlot[inSlot] != 0 ? mLinkSlot[inSlot] : mParent[inSlot]; }
		/// 父的世界矩阵乘到本地矩阵前面的那部分：有链接时是 链接槽位的 world * 链接矩阵，没有父时是单位矩阵
		JPH::DMat44				ParentWorld(int32_t inSlot) const;
		void					RebuildOrder();
		/// 写世界矩阵：和原来的不一样才写，并记下这个槽位变了
		/// 返回是不是变了（没变不写，也不重算渲染用的那份）
		bool					StoreWorld(int32_t inSlot, JPH::DMat44Arg inWorld);
		/// 重算一个槽位的世界矩阵，变了就写并把槽位记进 ioChanged（每个槽位每轮只记一次）；只碰这个槽位自己的数据，
		/// 同一层的槽位可以在几个线程里同时算，各用自己的 ioChanged
		void					UpdateOne(int32_t inSlot, std::vector<int32_t> &ioChanged);
		/// 由世界矩阵写渲染用的那份（平移减渲染原点）
		void					StoreRenderWorld(int32_t inSlot);
		/// 记下槽位变了（这一轮还没记过才加进列表）
		void					MarkChanged(int32_t inSlot);

		uint32_t				mCapacity;
		std::vector<uint8_t>	mAlive;
		std::vector<uint32_t>	mChildCount;
		std::vector<int32_t>	mFreeSlots;			///< 释放过的槽位，后进先出地重用
		std::vector<int32_t>	mLinkSlot;			///< 链接到的槽位，0 表示没有链接
		std::vector<float>		mLinkMatrix;		///< 链接矩阵，列主序 4x4
		std::vector<int32_t>	mLinked;			///< 有链接的槽位（释放槽位时去掉指向它的链接）
		uint32_t				mHighWater = 0;		///< 发出去过的最大槽位号
		std::vector<int32_t>	mOrder;				///< 活着的槽位，父节点在前
		std::vector<uint32_t>	mLevelEnd;			///< mOrder 里每个深度那一层的结束位置
		std::vector<int32_t>	mDepth;				///< 以下三个是 RebuildOrder 的临时数据，留着重用
		std::vector<int32_t>	mChain;
		std::vector<uint32_t>	mBuckets;
		bool					mOrderDirty = false;
		std::vector<uint32_t>	mChangedStamp;		///< 槽位上一次记进变了的列表时的轮次（去重用）
		uint32_t				mStamp = 1;			///< 这一轮（下一次 Update 之前）的轮次
		std::vector<int32_t>	mChanged;			///< 这一轮变了的槽位
		std::vector<int32_t>	mPublished;			///< 上一次 Update 交出去的
		JPH::DVec3				mRenderOrigin = JPH::DVec3::sZero();
	};

	/// 按句柄找变换存储，找不到时记下错误（提到 inWhat）。别的模块（物理）必须通过它找：句柄表是模板里的静态变量，
	/// 物理在 wasm 里是侧模块，有自己的一份，只有定义在这里（运行时模块）的这个函数用的是创建存储的那一份。
	TransformStore *LookupTransformStore(int32_t inHandle, const char *inWhat = "transform store");
}
