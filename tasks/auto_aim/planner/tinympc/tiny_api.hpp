// 头文件保护指令：防止同一头文件被多次包含，编译效率更高（GCC/Clang等主流编译器支持）
#pragma once

// 引入基础标准库：标准输入输出，用于接口内可能的日志打印、错误提示
#include <iostream>
// 引入自研ADMM算法核心头文件：该MPC库基于ADMM（交替方向乘子法）求解优化问题，底层算法实现依赖此文件
#include "admm.hpp"

// C/C++混合调用兼容宏：C++编译器编译时，将{}内代码按C语言规范编译，消除C++名字修饰
// 核心作用：让C语言项目、嵌入式裸机程序（多为C语言）能直接调用该MPC库的接口，同时C++项目也可无缝使用
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MPC求解器核心初始化接口，创建并初始化求解器实例
 * @details 初始化MPC离散线性系统模型、代价函数、ADMM超参数等核心配置，是所有接口的入口
 * @param solverp 输出参数：MPC求解器实例指针的指针，初始化后指向创建的TinySolver对象
 * @param Adyn 状态矩阵（nx×nx）：离散系统状态方程 x(k+1) = Adyn*x(k) + Bdyn*u(k) + fdyn 中的状态矩阵
 * @param Bdyn 输入矩阵（nx×nu）：系统状态方程中的控制输入矩阵
 * @param fdyn 偏移向量（nx×1）：系统状态方程中的常数偏移项
 * @param Q 状态代价矩阵（nx×nx）：代价函数中状态偏差的权重矩阵，Q=Q^T≥0，越大对状态偏差惩罚越重
 * @param R 输入代价矩阵（nu×nu）：代价函数中控制输入的权重矩阵，R=R^T>0，越大对控制量惩罚越重
 * @param rho ADMM算法超参数：惩罚因子，影响求解速度和收敛性
 * @param nx 状态量维度：系统状态变量的个数（如云台：yaw/pitch/角速度，nx=4）
 * @param nu 控制量维度：系统控制输入的个数（如云台：yaw/pitch控制量，nu=2）
 * @param N 预测步长：MPC向前预测的步数，N越大规划越精准但求解耗时越长
 * @param verbose 日志打印开关：0-关闭日志，1-开启日志（打印求解过程、收敛信息等）
 * @return int 执行状态：0表示成功，非0表示失败（如参数非法、内存分配失败）
 */
int tiny_setup(TinySolver** solverp,
                tinyMatrix Adyn, tinyMatrix Bdyn, tinyMatrix fdyn, tinyMatrix Q, tinyMatrix R, 
                tinytype rho, int nx, int nu, int N, int verbose);

/**
 * @brief 设置MPC状态量和控制量的边界约束（硬约束）
 * @details 限制状态量x和控制量u的取值范围，防止系统超出物理极限（如云台角度范围、电机转速限制）
 * @param solver 已初始化的MPC求解器实例指针
 * @param x_min 状态量下限（nx×1）：x的每个维度的最小值
 * @param x_max 状态量上限（nx×1）：x的每个维度的最大值
 * @param u_min 控制量下限（nu×1）：u的每个维度的最小值
 * @param u_max 控制量上限（nu×1）：u的每个维度的最大值
 * @return int 执行状态：0成功，非0失败（如求解器未初始化、矩阵维度不匹配）
 */
int tiny_set_bound_constraints(TinySolver* solver,
                    tinyMatrix x_min, tinyMatrix x_max,
                    tinyMatrix u_min, tinyMatrix u_max);

/**
 * @brief 设置MPC状态量和控制量的锥约束（如二阶锥SOC）
 * @details 处理系统的非线性约束，通过锥约束转化为ADMM可求解的凸优化问题（如距离约束、速度约束）
 * @param solver 已初始化的MPC求解器实例指针
 * @param Acu 控制量锥约束矩阵索引（整型向量）：标识控制量锥约束的矩阵结构
 * @param qcu 控制量锥约束参数（整型向量）：锥约束的核心参数
 * @param cu 控制量锥约束偏移向量：锥约束的常数项
 * @param Acx 状态量锥约束矩阵索引（整型向量）：标识状态量锥约束的矩阵结构
 * @param qcx 状态量锥约束参数（整型向量）：状态量锥约束核心参数
 * @param cx 状态量锥约束偏移向量：状态量锥约束的常数项
 * @return int 执行状态：0成功，非0失败
 */
int tiny_set_cone_constraints(TinySolver* solver,
                              VectorXi Acu, VectorXi qcu, tinyVector cu,
                              VectorXi Acx, VectorXi qcx, tinyVector cx);

/**
 * @brief 设置MPC状态量和控制量的线性等式/不等式约束
 * @details 处理系统的线性约束，如 x(k+1) = A*x(k) + B*u(k)、u ≤ a*x + b 等
 * @param solver 已初始化的MPC求解器实例指针
 * @param Alin_x 状态量线性约束矩阵：状态量的线性约束系数矩阵
 * @param blin_x 状态量线性约束偏移向量：状态量线性约束的常数项
 * @param Alin_u 控制量线性约束矩阵：控制量的线性约束系数矩阵
 * @param blin_u 控制量线性约束偏移向量：控制量线性约束的常数项
 * @return int 执行状态：0成功，非0失败
 */
int tiny_set_linear_constraints(TinySolver* solver,
                               tinyMatrix Alin_x, tinyVector blin_x,
                               tinyMatrix Alin_u, tinyVector blin_u);

/**
 * @brief 预计算并设置MPC求解缓存
 * @details 预计算系统模型、代价函数的固定参数（如矩阵乘积、逆矩阵）并存入缓存，避免实时求解时重复计算
 *          核心提升实时性，适配嵌入式10ms级控制周期
 * @param cache MPC缓存对象指针：存储预计算结果，供实时求解使用
 * @param Adyn/BDyn/fdyn/Q/R 同tiny_setup，系统模型和代价函数矩阵
 * @param nx/nu 状态量/控制量维度
 * @param rho ADMM超参数惩罚因子
 * @param verbose 日志打印开关
 * @return int 执行状态：0成功，非0失败
 */
int tiny_precompute_and_set_cache(TinyCache *cache, 
                                    tinyMatrix Adyn, tinyMatrix Bdyn, tinyMatrix fdyn, tinyMatrix Q, tinyMatrix R,
                                    int nx, int nu, tinytype rho, int verbose);

/**
 * @brief 计算ADMM算法的灵敏度矩阵
 * @details 为自适应rho（惩罚因子）提供梯度信息，实现rho的动态调整，提升MPC求解的收敛速度和稳定性
 * @param cache MPC缓存对象指针：预计算结果存储对象
 * @param Adyn/BDyn/Q/R 系统模型和代价函数矩阵
 * @param nx/nu 状态量/控制量维度
 * @param rho ADMM超参数惩罚因子
 * @param verbose 日志打印开关
 */
void compute_sensitivity_matrices(TinyCache *cache,
                                 tinyMatrix Adyn, tinyMatrix Bdyn, tinyMatrix Q, tinyMatrix R,
                                 int nx, int nu, tinytype rho, int verbose);

/**
 * @brief 根据灵敏度矩阵的导数更新MPC缓存中的矩阵
 * @details 配合自适应rho使用，当rho动态调整时，快速更新预计算缓存，无需重新调用tiny_precompute_and_set_cache
 * @param cache MPC缓存对象指针
 * @param delta_rho rho的变化量：当前rho与上一帧rho的差值
 * @return int 执行状态：0成功，非0失败
 */
int tiny_update_matrices_with_derivatives(TinyCache *cache, tinytype delta_rho);

/**
 * @brief MPC核心实时求解接口
 * @details 基于当前设置的初始状态、参考值、约束，调用ADMM算法求解最优控制序列
 *          求解完成后，最优控制量可从求解器实例中获取（如首步控制量u0）
 * @param solver 已初始化的MPC求解器实例指针
 * @return int 执行状态：0表示求解收敛成功，非0表示求解失败/不收敛（如迭代超上限、约束不满足）
 */
int tiny_solve(TinySolver *solver);

/**
 * @brief 更新MPC求解器的核心配置参数
 * @details 动态调整求解器的收敛阈值、迭代次数、约束使能等，无需重新初始化求解器
 * @param settings MPC配置对象指针
 * @param abs_pri_tol 原始残差收敛阈值：小于该值时，认为原始残差收敛
 * @param abs_dua_tol 对偶残差收敛阈值：小于该值时，认为对偶残差收敛
 * @param max_iter 最大迭代次数：ADMM算法的最大迭代步数，防止无限迭代
 * @param check_termination 收敛检查开关：0-不检查，1-检查（满足阈值则提前终止迭代）
 * @param en_state_bound 状态量边界约束使能：0-关闭，1-开启
 * @param en_input_bound 控制量边界约束使能：0-关闭，1-开启
 * @param en_state_soc 状态量二阶锥约束使能：0-关闭，1-开启
 * @param en_input_soc 控制量二阶锥约束使能：0-关闭，1-开启
 * @param en_state_linear 状态量线性约束使能：0-关闭，1-开启
 * @param en_input_linear 控制量线性约束使能：0-关闭，1-开启
 * @return int 执行状态：0成功，非0失败
 */
int tiny_update_settings(TinySettings* settings,
                            tinytype abs_pri_tol, tinytype abs_dua_tol, 
                            int max_iter, int check_termination, 
                            int en_state_bound, int en_input_bound,
                            int en_state_soc, int en_input_soc,
                            int en_state_linear, int en_input_linear);

/**
 * @brief 为MPC配置对象设置默认参数
 * @details 初始化时快速设置通用默认配置（如收敛阈值、迭代次数、约束默认关闭），简化配置流程
 * @param settings MPC配置对象指针
 * @return int 执行状态：0成功，非0失败
 */
int tiny_set_default_settings(TinySettings* settings);

/**
 * @brief 设置MPC求解的初始状态x0
 * @details 每帧实时更新，传入系统当前的实际状态（如云台当前的yaw/pitch/角速度），是MPC求解的起点
 * @param solver 已初始化的MPC求解器实例指针
 * @param x0 初始状态向量（nx×1）：系统当前的状态量
 * @return int 执行状态：0成功，非0失败（如维度不匹配）
 */
int tiny_set_x0(TinySolver* solver, tinyVector x0);

/**
 * @brief 设置MPC状态量的参考轨迹x_ref
 * @details 设置未来N步的状态参考值（nx×N），求解器将使系统状态尽可能逼近该参考轨迹
 *          （如云台跟踪目标时，未来N步的目标位姿即为x_ref）
 * @param solver 已初始化的MPC求解器实例指针
 * @param x_ref 状态参考矩阵（nx×N）：每一列对应未来一步的状态参考值
 * @return int 执行状态：0成功，非0失败
 */
int tiny_set_x_ref(TinySolver* solver, tinyMatrix x_ref);

/**
 * @brief 设置MPC控制量的参考轨迹u_ref
 * @details 设置未来N-1步的控制参考值（nu×(N-1)），可选配置，用于辅助代价函数优化
 * @param solver 已初始化的MPC求解器实例指针
 * @param u_ref 控制参考矩阵（nu×(N-1)）：每一列对应未来一步的控制参考值
 * @return int 执行状态：0成功，非0失败
 */
int tiny_set_u_ref(TinySolver* solver, tinyMatrix u_ref);

/**
 * @brief 初始化自适应rho的灵敏度矩阵
 * @details 为ADMM算法的自适应惩罚因子rho做准备，初始化相关的灵敏度矩阵内存和参数
 * @param solver Pointer to solver 已初始化的MPC求解器实例指针
 */
void tiny_initialize_sensitivity_matrices(TinySolver *solver);

/**
 * @brief 单独设置状态量的二阶锥（SOC）约束
 * @details 精细化配置接口，单独开启/配置状态量的二阶锥约束，无需调用通用的tiny_set_cone_constraints
 * @param solver 已初始化的MPC求解器实例指针
 * @param Acx 状态量锥约束矩阵（向量形式）：状态量SOC约束的系数向量
 * @param qcx 状态量锥约束参数向量：SOC约束的核心参数
 * @param cx 状态量锥约束偏移向量：SOC约束的常数项
 * @param numStateCones 状态量锥约束的个数：需要设置的SOC约束数量
 * @return int 执行状态：0成功，非0失败
 */
int tiny_setup_state_soc_constraints(TinySolver *solver, 
                                    tinyVector Acx, tinyVector qcx, tinyVector cx, 
                                    int numStateCones);
                                    
/**
 * @brief 单独设置控制量的二阶锥（SOC）约束
 * @details 精细化配置接口，单独开启/配置控制量的二阶锥约束，适配嵌入式场景的精细化控制需求
 * @param solver 已初始化的MPC求解器实例指针
 * @param Acu 控制量锥约束矩阵（向量形式）：控制量SOC约束的系数向量
 * @param qcu 控制量锥约束参数向量：SOC约束的核心参数
 * @param cu 控制量锥约束偏移向量：SOC约束的常数项
 * @param numInputCones 控制量锥约束的个数：需要设置的SOC约束数量
 * @return int 执行状态：0成功，非0失败
 */
int tiny_setup_input_soc_constraints(TinySolver *solver, 
                                    tinyVector Acu, tinyVector qcu, tinyVector cu, 
                                    int numInputCones);

// 结束C/C++混合调用兼容宏，与开头的extern "C"配对
#ifdef __cplusplus
}
#endif
