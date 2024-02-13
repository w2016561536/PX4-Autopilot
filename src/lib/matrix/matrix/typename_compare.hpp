#ifndef __HEADER_TYPE_NAME_COMPARE__
#define __HEADER_TYPE_NAME_COMPARE__

namespace Same{
   template<typename _Tp,_Tp _v>
   struct intergral_constant{
       static constexpr _Tp value = _v; //表示值
       typedef _Tp value_type; //值类型
       typedef intergral_constant<_Tp,_v> type; //表示自己的类型

       constexpr operator value_type () const noexcept {return value;} //
       constexpr value_type operator()() const noexcept {return value;} //since c++14
   };
    //定义true_type 和 false_type
    typedef intergral_constant<bool, true> true_type;
    typedef intergral_constant<bool, false> false_type;
    //is_same的实现
    template <typename ,typename >
    struct is_same: public false_type {};
    template <typename _Tp>
    struct is_same<_Tp,_Tp>: public true_type {};
}
#endif
