// 和 stride-ts 的属性类型布局相同的值类型：数据组件的列放进 wasm 内存时，C++ 直接按这些类型读写，
// JS 用 BigInt64Array / Int32Array 视图看同一块内存，不需要换算或拷贝。
// 布局同 Apache Arrow：decimal64、date32、time64[ns]、timestamp[ns]。
// 日期算法和 stride-ts 的 packages/engine/src/values/temporal.ts 相同（Howard Hinnant 的 days_from_civil），
// 两边的测试用同一组对照值。
#pragma once

#include <stdint.h>

/// 定点小数的单位数：值 = units / 10^scale。scale 是字段声明的一部分，不跟着值存。
typedef int64_t sn_decimal64;
/// 1970-01-01 起的天数。
typedef int32_t sn_date;
/// 当天零点起的纳秒。
typedef int64_t sn_time;
/// 1970-01-01T00:00 起的纳秒，不带时区的钟面时间。
typedef int64_t sn_datetime;
/// Unix 纪元（UTC）起的纳秒。
typedef int64_t sn_instant;

#define SN_NANOS_PER_SECOND INT64_C(1000000000)
#define SN_NANOS_PER_DAY (INT64_C(86400) * SN_NANOS_PER_SECOND)

/// 向下取整的除法（C 的 / 向零截断，负数要另外处理）。
static inline int64_t sn_floor_div(int64_t value, int64_t divisor)
{
	int64_t quotient = value / divisor;
	return (value % divisor != 0 && ((value < 0) != (divisor < 0))) ? quotient - 1 : quotient;
}

/// 公历年月日 → 1970-01-01 起的天数。
static inline sn_date sn_date_from_civil(int32_t year, int32_t month, int32_t day)
{
	const int32_t y = month <= 2 ? year - 1 : year;
	const int32_t era = (int32_t)sn_floor_div(y, 400);
	const int32_t year_of_era = y - era * 400;
	const int32_t day_of_year = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
	const int32_t day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
	return era * 146097 + day_of_era - 719468;
}

/// 1970-01-01 起的天数 → 公历年月日。
static inline void sn_date_to_civil(sn_date days, int32_t *out_year, int32_t *out_month, int32_t *out_day)
{
	const int32_t z = days + 719468;
	const int32_t era = (int32_t)sn_floor_div(z, 146097);
	const int32_t day_of_era = z - era * 146097;
	const int32_t year_of_era = (day_of_era - day_of_era / 1460 + day_of_era / 36524 - day_of_era / 146096) / 365;
	const int32_t day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
	const int32_t mp = (5 * day_of_year + 2) / 153;
	const int32_t day = day_of_year - (153 * mp + 2) / 5 + 1;
	const int32_t month = mp < 10 ? mp + 3 : mp - 9;
	*out_year = year_of_era + era * 400 + (month <= 2 ? 1 : 0);
	*out_month = month;
	*out_day = day;
}

/// 日期加当天的时刻 → 钟面时间。
static inline sn_datetime sn_datetime_from(sn_date date, sn_time time)
{
	return (int64_t)date * SN_NANOS_PER_DAY + time;
}

/// 钟面时间的日期部分（负数也向下取整，1969-12-31T23:00 的日期是 -1）。
static inline sn_date sn_datetime_date(sn_datetime value)
{
	return (sn_date)sn_floor_div(value, SN_NANOS_PER_DAY);
}

/// 钟面时间的时刻部分。
static inline sn_time sn_datetime_time(sn_datetime value)
{
	return value - (int64_t)sn_datetime_date(value) * SN_NANOS_PER_DAY;
}

/// 小数换算位数：位数增加时精确；减少时按银行家舍入（同 stride-ts Decimal 的默认 half-even）。
/// 结果超出 int64 时行为未定义，调用方按字段声明的精度（最多 18 位）保证不会。
static inline sn_decimal64 sn_decimal64_rescale(sn_decimal64 units, int32_t from_scale, int32_t to_scale)
{
	if (to_scale >= from_scale)
	{
		for (int32_t i = from_scale; i < to_scale; ++i)
			units *= 10;
		return units;
	}
	int64_t divisor = 1;
	for (int32_t i = to_scale; i < from_scale; ++i)
		divisor *= 10;
	int64_t quotient = units / divisor;
	int64_t remainder = units % divisor;
	if (remainder == 0)
		return quotient;
	const int64_t twice = (remainder < 0 ? -remainder : remainder) * 2;
	const int64_t away = units < 0 ? quotient - 1 : quotient + 1;
	if (twice > divisor || (twice == divisor && quotient % 2 != 0))
		return away;
	return quotient;
}
