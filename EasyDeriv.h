#pragma once

#ifdef ED_WITH_CERES
// 为了兼容 ceres::Jet 类型，需要定义一些额外的函数
#define ED_CERES_PATCH namespace std					\
{														\
	template <typename T, int N>						\
	auto pow(const ceres::Jet<T, N>& a, const double b)	\
	{													\
		return std::pow(a, ceres::Jet<T, N>(b));		\
	}													\
}
#define ED_FORCE_USE_DOUBLE
#endif

#include <concepts>
#include <tuple>
#include <type_traits>
#include <cmath>
#include <vector>
#include <array>

namespace EasyDeriv
{
#if !defined(ED_FORCE_USE_DOUBLE)
	using FloatType = float;
#elif !defined(ED_USE_TYPE_FLOAT)
	using FloatType = double;
#endif

	// 辅助函数模板，用于检查元组的所有元素是否都是 float 类型
	template <typename T>
	struct is_all_float_tuple : std::false_type {};

	template <typename... Ts>
	struct is_all_float_tuple<std::tuple<Ts...>> :
		std::conjunction<std::is_same<Ts, FloatType>...> {};

	// 定义 VarType 概念
	template <typename T>
	concept VarType =
		// 检查是否为元素全是 FloatType 的 std::tuple
		requires {
			requires std::is_same_v<T, std::remove_cvref_t<T>>;
			typename std::tuple_size<T>::type;
			requires is_all_float_tuple<T>::value;
		}
		// 检查是否可以被索引
		|| requires(T t, size_t ind) {
			requires std::is_same_v<T, std::remove_cvref_t<T>>;
			{ t[ind] } -> std::convertible_to<FloatType>;
		}
		// 兼容 ceres::Jet 类型
		|| requires(T t, size_t ind) {
			{ t[ind].a } -> std::convertible_to<FloatType>;
		};

	template <size_t ID>
	struct Variable
	{
		Variable() = delete;
		static constexpr size_t id = ID;
		static inline auto evaluate(const auto& vars)
		{
			return vars[ID];
		}
		template <typename... Args>
		static inline constexpr FloatType evaluate(const std::tuple<Args...> &vars)
		{
			return get<ID>(vars);
		}
		template <size_t N>
		static inline constexpr FloatType evaluate(const std::array<FloatType, N>& vars)
		{
			return vars[ID];
		}
		static inline constexpr FloatType evaluate(const std::vector<FloatType>& vars)
		{
			return vars[ID];
		}
		static inline constexpr FloatType evaluate(const double* const vars)
		{
			return FloatType(vars[ID]);
		}
		static inline constexpr FloatType evaluate(const float* const vars)
		{
			return FloatType(vars[ID]);
		}
	};

	template <typename T>
	struct is_variable : std::false_type {};
	template <int ID>
	struct is_variable<Variable<ID>> : std::true_type {};

	template <typename T>
	concept VariableType = is_variable<T>::value;

	template <typename... Exprs>
	struct ExpressionPack
	{
		constexpr static size_t size = sizeof...(Exprs);
		template <typename... Args>
		static inline constexpr auto evaluate(const std::tuple<Args...> &vars)
		{
			static_assert(sizeof...(Args) == size);
			return std::make_tuple(Exprs::evaluate(vars)...);
		}
		template <size_t N>
		static inline constexpr auto evaluate(const std::array<FloatType, N>& vars)
		{
			static_assert(N == size);
			return std::array<FloatType, size>{Exprs::evaluate(vars)...};
		}
		static inline constexpr auto evaluate(const std::vector<FloatType>& vars)
		{
			std::vector<FloatType> result(size);
			for (size_t i = 0; i < size; ++i)
				result[i] = Exprs::evaluate(vars);
			return result;
		}
		static inline constexpr auto evaluate(const auto* const vars)
		{
			std::vector<FloatType> result(size);
			for (size_t i = 0; i < size; ++i)
				result[i] = Exprs::evaluate(vars);
			return result;
		}
	};

	template <typename T>
	struct is_expression_pack : std::false_type {};
	template <typename... Exprs>
	struct is_expression_pack<ExpressionPack<Exprs...>> : std::true_type {};

	template <typename T>
	concept ExpressionPackType = is_expression_pack<T>::value;

	template <typename T>
	struct is_nonexpression_pack : std::true_type {};
	template <typename... Exprs>
	struct is_nonexpression_pack<ExpressionPack<Exprs...>> : std::false_type {};

	template <typename T>
	concept NonExpressionPackType = is_nonexpression_pack<T>::value;

	template <VariableType... Vars>
	using VariablePack = ExpressionPack<Vars...>;

	template <typename T>
	struct is_variable_pack : std::false_type {};
	template <VariableType... Vars>
	struct is_variable_pack<VariablePack<Vars...>> : std::true_type {};

	template <typename T>
	concept VariablePackType = is_variable_pack<T>::value;

	template <size_t StartIdx, size_t EndIdx, typename... Generated>
	struct CreateVariablePack
	{
		CreateVariablePack() = delete;
		using type = typename CreateVariablePack<StartIdx + 1, EndIdx, Generated..., Variable<StartIdx>>::type;
	};

	template <size_t EndIdx, typename... Generated>
	struct CreateVariablePack<EndIdx, EndIdx, Generated...>
	{
		CreateVariablePack() = delete;
		using type = VariablePack<Generated...>;
	};

	template <size_t MaxN, size_t UsedN = 0>
	struct VariableCreator
	{
		VariableCreator() = delete;
		static_assert(UsedN <= MaxN);
		template <size_t N>
		struct Create
		{
			using pack = typename CreateVariablePack<UsedN, UsedN + N>::type;
			using next = VariableCreator<MaxN, UsedN + N>;
		};
	};

	template <FloatType C>
	struct _Const
	{
		template <typename T>
		static inline constexpr FloatType evaluate(const T &vars)
		{
			return C;
		}
	};

	template <typename T>
	struct is_const : std::false_type {};
	template <FloatType C>
	struct is_const<_Const<C>> : std::true_type {};
	template <typename T>
	concept ConstType = is_const<T>::value;

	template <typename T>
	struct is_non_const : std::true_type {};
	template <FloatType C>
	struct is_non_const<_Const<C>> : std::false_type {};

	template <auto C>
	using Const = _Const<FloatType(C)>;

	template <typename T>
	concept NonConstType = is_non_const<T>::value;

	namespace ConstSyms
	{
		using Zero = Const<0.>;
		using One = Const<1.>;
		using NegOne = Const<-1.>;
		using Pi = Const<3.14159265358979323846>;
		using E = Const<2.71828182845904523536>;
	}

	template <typename Left, typename Right>
	struct Add
	{
		// 默认不优化
		using optim = Add<Left, Right>;
		template <VarType T>
		static inline constexpr auto evaluate(const T &vars)
		{
			return Left::evaluate(vars) + Right::evaluate(vars);
		}
	};

	template <typename Left, typename Right>
	struct Subtract
	{
		using optim = Subtract<Left, Right>;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return Left::evaluate(vars) - Right::evaluate(vars);
		}
	};

	template <typename Left, typename Right>
	struct Power
	{
		using optim = Power<Left, Right>;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return pow(Left::evaluate(vars), Right::evaluate(vars));
		}
	};

	template <typename Left, typename Right>
	struct Multiply
	{
		using optim = Multiply<Left, Right>;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return Left::evaluate(vars) * Right::evaluate(vars);
		}
	};

	template <typename Left, typename Right>
	struct Divide
	{
		using optim = Divide<Left, Right>;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return Left::evaluate(vars) / Right::evaluate(vars);
		}
	};

	template <typename Left>
	struct Sin
	{
		using optim = Sin<Left>;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return std::sin(Left::evaluate(vars));
		}
	};

	template <typename Left>
	struct Cos
	{
		using optim = Cos<Left>;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return std::cos(Left::evaluate(vars));
		}
	};

	template <typename Expr, size_t VarId>
	struct Derivative
	{
		using type = ConstSyms::Zero;
	};

	// 变量偏导数
	template <size_t VarId>
	struct Derivative<Variable<VarId>, VarId>
	{
		using type = ConstSyms::One;
	};

	template <size_t VarId, size_t OtherId>
	struct Derivative<Variable<OtherId>, VarId>
	{
		using type = ConstSyms::Zero;
	};

	// 向量累加
	template <ExpressionPackType ExprPack>
	struct AccumulateImpl;
	template <typename Expr1, typename... Exprs>
	struct AccumulateImpl<ExpressionPack<Expr1, Exprs...>>
	{
		using type = Add<Expr1, typename AccumulateImpl<ExpressionPack<Exprs...>>::type>::optim;
	};
	template <>
	struct AccumulateImpl<ExpressionPack<>>
	{
		using type = ConstSyms::Zero;
	};
	template <ExpressionPackType ExprPack>
	using Accumulate = AccumulateImpl<ExprPack>::type;

	// 向量点乘
	template <typename Left, typename Right>
	struct DotImpl;
	template <typename... LeftExprs, typename... RightExprs>
	struct DotImpl<ExpressionPack<LeftExprs...>, ExpressionPack<RightExprs...>>
	{
		static_assert(sizeof...(LeftExprs) == sizeof...(RightExprs));
		using type = Accumulate<ExpressionPack<typename Multiply<LeftExprs, RightExprs>::optim...>>;
	};
	// 向量缩放
	template <NonExpressionPackType Left, typename... RightExprs>
	struct DotImpl<Left, ExpressionPack<RightExprs...>>
	{
		using type = ExpressionPack<typename Multiply<Left, RightExprs>::optim...>;
	};
	template <typename... LeftExprs, NonExpressionPackType Right>
	struct DotImpl<ExpressionPack<LeftExprs...>, Right>
	{
		using type = typename DotImpl<Right, ExpressionPack<LeftExprs...>>::type;
	};
	template <typename Left, typename Right>
	using Dot = typename DotImpl<Left, Right>::type;

	// 向量相加
	template <ExpressionPackType Left, ExpressionPackType Right>
	struct AddPackImpl;
	template <typename... LeftExprs, typename... RightExprs>
	struct AddPackImpl<ExpressionPack<LeftExprs...>, ExpressionPack<RightExprs...>>
	{
		static_assert(sizeof...(LeftExprs) == sizeof...(RightExprs));
		using type = ExpressionPack<typename Add<LeftExprs, RightExprs>::optim...>;
	};
	template <ExpressionPackType Left, ExpressionPackType Right>
	using AddPack = typename AddPackImpl<Left, Right>::type;

	// 向量相减
	template <ExpressionPackType Left, ExpressionPackType Right>
	struct SubtractPack;
	template <typename... LeftExprs, typename... RightExprs>
	struct SubtractPack<ExpressionPack<LeftExprs...>, ExpressionPack<RightExprs...>>
		: ExpressionPack<typename Subtract<LeftExprs, RightExprs>::optim...>
	{
		static_assert(sizeof...(LeftExprs) == sizeof...(RightExprs));
	};

	// 哈达玛积
	template <VariablePackType Left, VariablePackType Right>
	struct HadamardProduct;
	template <typename... LeftExprs, typename... RightExprs>
	struct HadamardProduct<ExpressionPack<LeftExprs...>, ExpressionPack<RightExprs...>>
		: ExpressionPack<typename Multiply<LeftExprs, RightExprs>::optim...>
	{
		static_assert(sizeof...(LeftExprs) == sizeof...(RightExprs));
	};

	// Sin函数
	template <VariablePackType Left>
	struct VecSin;
	template <typename... LeftExprs>
	struct VecSin<ExpressionPack<LeftExprs...>>
		: ExpressionPack<typename Sin<LeftExprs>::optim...>
	{
	};

	// Cos函数
	template <VariablePackType Left>
	struct VecCos;
	template <typename... LeftExprs>
	struct VecCos<ExpressionPack<LeftExprs...>>
		: ExpressionPack<typename Cos<LeftExprs>::optim...>
	{
	};

	// 常数偏导数
	template <FloatType C, size_t VarId>
	struct Derivative<_Const<C>, VarId>
	{
		using type = ConstSyms::Zero;
	};

	// 加法偏导数
	template <typename Left, typename Right, size_t VarId>
	struct Derivative<Add<Left, Right>, VarId>
	{
		using type = Add<
			typename Derivative<Left, VarId>::type,
			typename Derivative<Right, VarId>::type>::optim;
	};

	// 减法偏导数
	template <typename Left, typename Right, size_t VarId>
	struct Derivative<Subtract<Left, Right>, VarId>
	{
		using type = Subtract<
			typename Derivative<Left, VarId>::type,
			typename Derivative<Right, VarId>::type>::optim;
	};

	// 乘法偏导数
	template <typename Left, typename Right, size_t VarId>
	struct Derivative<Multiply<Left, Right>, VarId>
	{
		using type = Add<
			typename Multiply<typename Derivative<Left, VarId>::type, Right>::optim,
			typename Multiply<Left, typename Derivative<Right, VarId>::type>::optim>::optim;
	};

	// 除法偏导数
	template <typename Left, typename Right, size_t VarId>
	struct Derivative<Divide<Left, Right>, VarId>
	{
		using numerator = Subtract<
			typename Multiply<typename Derivative<Left, VarId>::type, Right>::optim,
			typename Multiply<Left, typename Derivative<Right, VarId>::type>::optim>;
		using denominator = Multiply<Right, Right>;
		using type = Divide<
			typename numerator::optim,
			typename denominator::optim>::optim;
	};

	// 幂函数偏导数
	template <typename Left, typename Right, size_t VarId>
	struct Derivative<Power<Left, Right>, VarId>
	{
		using type = Multiply<
			typename Multiply<Right,
							  typename Power<Left, Subtract<Right, ConstSyms::One>>::optim>::optim,
			typename Derivative<Left, VarId>::type>::optim;
	};

	// 提取偏导数
	template <typename Expr, size_t VarId>
	using Derivative_t = typename Derivative<Expr, VarId>::type;
}

#pragma region 化简规则
namespace EasyDeriv
{
	//C1 + C2
	template <FloatType C1, FloatType C2>
	struct Add<_Const<C1>, _Const<C2>>
	{
		using optim = Const<C1 + C2>;
		template <typename T>
		static inline constexpr FloatType evaluate(const T &vars)
		{
			return C1 + C2;
		}
	};
	//x + 0 = x
	template <NonConstType Left>
	struct Add<Left, ConstSyms::Zero>
	{
		using optim = Left;
		template <VarType T>
		static inline constexpr FloatType evaluate(const T& vars)
		{
			return Left::evaluate(vars);
		}
	};
	//0 + x = x
	template <NonConstType Right>
	struct Add<ConstSyms::Zero, Right> : Add<Right, ConstSyms::Zero>
	{
	};

	//C1 - C2
	template <FloatType C1, FloatType C2>
	struct Subtract<_Const<C1>, _Const<C2>>
	{
		using optim = Const<C1 - C2>;
		template <typename T>
		static inline constexpr FloatType evaluate(const T& vars)
		{
			return C1 - C2;
		}
	};
	//x - 0 = x
	template <NonConstType Left>
	struct Subtract<Left, ConstSyms::Zero>
	{
		using optim = Left;
		template <VarType T>
		static inline constexpr FloatType evaluate(const T& vars)
		{
			return Left::evaluate(vars);
		}
	};
	//x - x = 0
	template <NonConstType Left>
	struct Subtract<Left, Left>
	{
		using optim = ConstSyms::Zero;
		template <typename T>
		static inline constexpr FloatType evaluate(const T &vars)
		{
			return 0.0;
		}
	};

	//C1 ^ C2
	template <FloatType C1, FloatType C2>
	struct Power<_Const<C1>, _Const<C2>>
	{
		using optim = Const<std::pow(C1, C2)>;
		template <typename T>
		static inline constexpr FloatType evaluate(const T &vars)
		{
			return std::pow(C1, C2);
		}
	};
	//x ^ 0 = 1
	template <NonConstType Left>
	struct Power<Left, ConstSyms::Zero>
	{
		using optim = ConstSyms::One;
		template <typename T>
		static inline constexpr FloatType evaluate(const T &vars)
		{
			return 1.0;
		}
	};
	//x ^ 1 = x
	template <NonConstType Left>
	struct Power<Left, ConstSyms::One>
	{
		using optim = Left;
		template <VarType T>
		static inline constexpr FloatType evaluate(const T& vars)
		{
			return Left::evaluate(vars);
		}
	};
	//0 ^ x = 0, 1 ^ x = 1
	template <FloatType C1, NonConstType Right>
	struct Power<_Const<C1>, Right>
	{
		using optim = std::conditional_t<
			C1 == 0, ConstSyms::Zero,
			std::conditional_t<
				C1 == 1, ConstSyms::One,
				Power<Const<C1>, Right>>>;
		template <typename T>
		static inline constexpr FloatType evaluate(const T &vars)
		{
			if constexpr (C1 == 0) return 0.0;
			else if (C1 == 1) return 1.0;
		}
	};
	// x ^ 2 = x * x ?

	//C1 * C2
	template <FloatType C1, FloatType C2>
	struct Multiply<_Const<C1>, _Const<C2>>
	{
		using optim = Const<C1 * C2>;
		template <typename T>
		static inline constexpr FloatType evaluate(const T &vars)
		{
			return C1 * C2;
		}
	};
	//0 * x = 0, 1 * x = x
	template <FloatType C1, NonConstType Right>
	struct Multiply<_Const<C1>, Right>
	{
		using optim = std::conditional_t<
			C1 == 0.0,
			ConstSyms::Zero,
			std::conditional_t<
				C1 == 1.0,
				Right,
				Multiply<Const<C1>, Right>>>;
		template <VarType T>
		static inline constexpr auto evaluate(const T &vars)
		{
			if constexpr (C1 == 0) return 0.0;
			else if (C1 == 1) return Right::evaluate(vars);
			else return C1 * Right::evaluate(vars);
		}
	};
	//x * 0 = 0, x * 1 = x
	template <NonConstType Left, FloatType C2>
	struct Multiply<Left, _Const<C2>> : Multiply<_Const<C2>, Left>
	{
	};
	//x * x = x^2
	template <NonConstType Left>
	struct Multiply<Left, Left>
	{
		using optim = Power<Left, Const<2.>>::optim;
		template <VarType T>
		static inline constexpr auto evaluate(const T &vars)
		{
			return Left::evaluate(vars) * Left::evaluate(vars);
		}
	};
	//x ^ n * x = x ^ (n + 1)
	template <NonConstType Left, FloatType CN>
	struct Multiply<Power<Left, _Const<CN>>, Left>
	{
		using optim = Power<Left, Const<CN + 1>>;
		template <VarType T>
		static inline constexpr auto evaluate(const T &vars)
		{
			return std::pow(Left::evaluate(vars), CN + 1);
		}
	};
	//x * x ^ n = x ^ (n + 1)
	template <NonConstType Left, FloatType CN>
	struct Multiply<Left, Power<Left, _Const<CN>>>
		: Multiply<Power<Left, _Const<CN>>, Left>
	{
	};

	//C1 / C2
	template <FloatType C1, FloatType C2>
	struct Divide<_Const<C1>, _Const<C2>>
	{
		using optim = Const<C1 / C2>;
		template <typename T>
		static inline constexpr auto evaluate(const T &vars)
		{
			return C1 / C2;
		}
	};
	//0 / x = 0
	template <NonConstType Right>
	struct Divide<ConstSyms::Zero, Right>
	{
		using optim = ConstSyms::Zero;
		template <typename T>
		static inline constexpr auto evaluate(const T &vars)
		{
			return 0.0;
		}
	};
	//x / 1 = x
	template <NonConstType Left>
	struct Divide<Left, ConstSyms::One>
	{
		using optim = Left;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return Left::evaluate(vars);
		}
	};
	//x / x = 1
	template <NonConstType Left>
	struct Divide<Left, Left>
	{
		using optim = ConstSyms::One;
		template <typename T>
		static inline constexpr auto evaluate(const T &vars)
		{
			return 1.0;
		}
	};
	//x ^ n / x = x ^ (n - 1)
	template <NonConstType Left, FloatType CN>
	struct Divide<Power<Left, _Const<CN>>, Left>
	{
		using optim = Power<Left, _Const<CN - 1>>::optim;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return std::pow(Left::evaluate(vars), CN - 1);
		}
	};
	//x / x ^ n = x ^ (1 - n)
	template <NonConstType Left, FloatType CN>
	struct Divide<Left, Power<Left, _Const<CN>>>
	{
		using optim = Power<Left, Const<1 - CN>>::optim;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return std::pow(Left::evaluate(vars), 1 - CN);
		}
	};

	//x ^ -1 = 1 / x
	template <NonConstType Left>
	struct Power<Left, ConstSyms::NegOne>
	{
		using optim = Divide<ConstSyms::One, Left>;
		template <VarType T>
		static inline constexpr auto evaluate(const T& vars)
		{
			return 1.0 / Left::evaluate(vars);
		}
	};
}
#pragma endregion 化简规则

#pragma region 测试用例
namespace EasyDeriv
{
	static constexpr std::tuple<FloatType, FloatType, FloatType> tuple_vars = {
		FloatType(1.0), FloatType(2.0), FloatType(3.0)
	};
	static constexpr std::array<FloatType, 3> std_array_vars = {
		FloatType(1.0), FloatType(2.0), FloatType(3.0)
	};
	static std::vector<FloatType> vector_vars = {
		FloatType(1.0), FloatType(2.0), FloatType(3.0)
	};
	static constexpr float const float_array_vars[] = {
		FloatType(1.0), FloatType(2.0), FloatType(3.0)
	};
	static constexpr double const double_array_vars[] = {
		FloatType(1.0), FloatType(2.0), FloatType(3.0)
	};
	static constexpr const double* const double_pointer_vars = const_cast<double*>(double_array_vars);
	static constexpr const float* const float_pointer_vars = const_cast<float*>(float_array_vars);
	static_assert(std::is_same_v<Derivative_t<Variable<0>, 0>, ConstSyms::One>);
	static_assert(std::is_same_v<Derivative_t<Variable<0>, 1>, ConstSyms::Zero>);
	static_assert(std::is_same_v<Derivative_t<Variable<1>, 0>, ConstSyms::Zero>);
	static_assert(std::is_same_v<Derivative_t<ConstSyms::One, 0>, ConstSyms::Zero>);
	static_assert(std::is_same_v<Derivative_t<Add<Variable<0>, Variable<1>>, 0>, ConstSyms::One>);
	static_assert(std::is_same_v<Derivative_t<Add<Variable<0>, Variable<1>>, 1>, ConstSyms::One>);
	static_assert(std::is_same_v<Derivative_t<Add<Variable<0>, Variable<1>>, 2>, ConstSyms::Zero>);
	static_assert(std::is_same_v<decltype(
		Add<Variable<0>, Variable<1>>::evaluate(tuple_vars)), FloatType>);
	static_assert( // 1 + 2 = 3
		Add<Variable<0>, Variable<1>>::evaluate(tuple_vars) == 3);
	static_assert(std::is_same_v<decltype(
		Add<Variable<0>, Variable<1>>::evaluate(std_array_vars)), FloatType>);
	static_assert( // 1 + 2 = 3
		Add<Variable<0>, Variable<1>>::evaluate(std_array_vars) == 3);
	static_assert(std::is_same_v<decltype(
		Add<Variable<0>, Variable<1>>::evaluate(vector_vars)), FloatType>);
	static_assert(std::is_same_v<decltype(
		Add<Variable<0>, Variable<1>>::evaluate(double_array_vars)), FloatType>);
	static_assert(std::is_same_v<decltype(
		Add<Variable<0>, Variable<1>>::evaluate(float_array_vars)), FloatType>);
	static_assert(std::is_same_v<decltype(
		Add<Variable<0>, Variable<1>>::evaluate(double_pointer_vars)), FloatType>);
	static_assert(std::is_same_v<decltype(
		Add<Variable<0>, Variable<1>>::evaluate(float_pointer_vars)), FloatType>);
	static_assert( // 1 + 2 = 3
		Add<Variable<0>, Variable<1>>::evaluate(double_array_vars) == 3);
	static_assert( // 1 + 2 = 3
		Add<Variable<0>, Variable<1>>::evaluate(float_array_vars) == 3);
	static_assert(std::is_same_v<Derivative_t<Multiply<Variable<0>, Variable<1>>, 0>, Variable<1>>);
	static_assert(std::is_same_v<Derivative_t<Multiply<Variable<0>, Variable<1>>, 1>, Variable<0>>);
	static_assert(std::is_same_v<Derivative_t<Multiply<Variable<0>, Variable<1>>, 2>, ConstSyms::Zero>);
	static_assert(std::is_same_v<decltype(
		Derivative_t<Multiply<Variable<0>, Variable<1>>, 2>::evaluate(tuple_vars)), FloatType>);
	static_assert(std::is_same_v<decltype(
		Multiply<Variable<0>, Variable<1>>::evaluate(std_array_vars)), FloatType>);
	static_assert(std::is_same_v<decltype(
		Multiply<Variable<0>, Variable<1>>::evaluate(double_array_vars)), FloatType>);
	static_assert(std::is_same_v<decltype(
		Multiply<Variable<0>, Variable<1>>::evaluate(float_array_vars)), FloatType>);
	static_assert(std::is_same_v<Derivative_t<Divide<Variable<0>, Variable<1>>, 0>, Divide<ConstSyms::One, Variable<1>>>);
	static_assert(std::is_same_v<Derivative_t<Divide<Variable<0>, Variable<1>>, 2>, ConstSyms::Zero>);
	static_assert(std::is_same_v<Derivative_t<Divide<Variable<0>, ConstSyms::One>, 0>, ConstSyms::One>);
	static_assert(std::is_same_v<Derivative_t<Divide<Variable<0>, ConstSyms::One>, 1>, ConstSyms::Zero>);
	static_assert(std::is_same_v<Derivative_t<Divide<Variable<0>, ConstSyms::One>, 2>, ConstSyms::Zero>);
	static_assert(std::is_same_v<Derivative_t<Divide<ConstSyms::One, Variable<1>>, 0>, ConstSyms::Zero>);
	static_assert(std::is_same_v<Derivative_t<Divide<ConstSyms::One, Variable<1>>, 2>, ConstSyms::Zero>);
	static_assert(std::is_same_v<Derivative_t<Divide<ConstSyms::Zero, Variable<1>>, 0>, ConstSyms::Zero>);
}
#pragma endregion 测试用例
