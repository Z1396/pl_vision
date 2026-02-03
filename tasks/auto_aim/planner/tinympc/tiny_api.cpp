// 引入自研MPC库头文件：包含接口声明、自定义类型（TinySolver/tinyMatrix等）定义
#include "tiny_api.hpp"
// 引入常量定义头文件：包含MPC求解器默认配置常量（如默认收敛阈值、最大迭代次数）
#include "tiny_api_constants.hpp"

// 引入C++标准输入输出库：用于维度校验错误提示、日志打印
#include <iostream>

// C/C++混合调用兼容宏：保证C语言编译器可正确解析函数名，消除C++名字修饰
#ifdef __cplusplus
extern "C" {
#endif

// 引入Eigen命名空间：简化矩阵/向量操作代码，Eigen是底层核心线性代数库
using namespace Eigen;
// 定义Eigen矩阵格式化输出规则：保留4位小数、无行首空格、元素间用逗号分隔、行尾换行、前后加[]
IOFormat TinyApiFmt(4, 0, ", ", "\n", "[", "]");

/**
 * @brief 矩阵/向量维度校验工具函数
 * @details 检查目标矩阵/向量的行/列数是否符合预期，不一致则打印错误信息并返回错误码
 * @param matrix_name 矩阵/向量名称（用于错误提示）
 * @param rows_or_columns 校验维度类型（"rows"表示行，"columns"表示列）
 * @param actual 实际行/列数
 * @param expected 预期行/列数
 * @return int 校验结果：0-成功，1-失败
 */
static int check_dimension(std::string matrix_name, std::string rows_or_columns, int actual, int expected) {
    if (actual != expected) {
        // 打印维度不匹配错误信息，包含实际值和预期值
        std::cout << matrix_name << " has " << actual << " " << rows_or_columns << ". Expected " << expected << "." << std::endl;
        return 1;
    }
    return 0;
}

/**
 * @brief MPC求解器核心初始化接口实现
 * @details 创建并初始化TinySolver及其所有子模块（Solution/Cache/Settings/Workspace），
 *          完成维度校验、内存分配、默认参数设置、工作空间初始化，是所有接口的入口
 * @param solverp 输出参数：求解器实例指针的指针，初始化后指向新创建的TinySolver对象
 * @param Adyn 状态转移矩阵(nx×nx)：离散系统 x(k+1)=Adyn*x(k)+Bdyn*u(k)+fdyn
 * @param Bdyn 输入矩阵(nx×nu)：系统控制输入矩阵
 * @param fdyn 仿射偏移向量(nx×1)：系统常数偏移项
 * @param Q 状态代价矩阵(nx×nx)：代价函数中状态偏差的权重矩阵（Q=Q^T≥0）
 * @param R 控制代价矩阵(nu×nu)：代价函数中控制输入的权重矩阵（R=R^T>0）
 * @param rho ADMM算法惩罚因子：影响求解收敛速度和稳定性
 * @param nx 状态量维度：系统状态变量个数（如云台：yaw/pitch/角速度，nx=4）
 * @param nu 控制量维度：系统控制输入个数（如云台：yaw/pitch控制量，nu=2）
 * @param N 预测步长：MPC向前预测步数（N越大规划越精准，求解耗时越长）
 * @param verbose 日志打印开关：0-关闭，1-开启（预计算阶段打印矩阵信息）
 * @return int 执行状态：0-成功，非0-维度校验失败（错误码为各维度校验结果的或值）
 */
int tiny_setup(TinySolver** solverp,
                tinyMatrix Adyn, tinyMatrix Bdyn, tinyMatrix fdyn, tinyMatrix Q, tinyMatrix R, 
                tinytype rho, int nx, int nu, int N, int verbose) {

    // 1. 为MPC求解器所有子模块分配内存：动态创建堆对象，保证生命周期与求解器一致
    TinySolution *solution = new TinySolution();  // 求解结果存储模块：保存最优状态/控制序列
    TinyCache *cache = new TinyCache();          // 预计算缓存模块：存储Riccati递推结果，避免实时求解重复计算
    TinySettings *settings = new TinySettings();  // 求解器配置模块：保存收敛阈值、迭代次数、约束使能等
    TinyWorkspace *work = new TinyWorkspace();    // 核心工作空间：保存系统参数、中间变量、约束条件等
    TinySolver *solver = new TinySolver();        // MPC求解器主对象：聚合所有子模块

    // 2. 关联求解器主对象与各子模块：建立层级引用关系
    solver->solution = solution;
    solver->cache = cache;
    solver->settings = settings;
    solver->work = work;

    // 3. 将初始化后的求解器指针赋值给输出参数，供外部调用者持有
    *solverp = solver;

    // 4. 初始化求解结果模块：置零迭代次数、求解状态，初始化最优状态/控制序列为零矩阵
    solution->iter = 0;          // 迭代次数初始化为0
    solution->solved = 0;        // 求解状态初始化为0（0-未求解，1-求解成功）
    solution->x = tinyMatrix::Zero(nx, N);    // 最优状态序列(nx×N)：初始化为零
    solution->u = tinyMatrix::Zero(nu, N-1);  // 最优控制序列(nu×N-1)：初始化为零

    // 5. 初始化求解器配置模块：设置默认收敛阈值、最大迭代次数、约束使能等
    tiny_set_default_settings(settings);

    // 6. 初始化核心工作空间：记录系统核心维度参数（状态/控制量维度、预测步长）
    work->nx = nx;
    work->nu = nu;
    work->N = N;

    // 7. 严格校验输入矩阵/向量的维度：保证与系统参数(nx/nu)匹配，避免后续计算维度错误
    int status = 0;  // 维度校验状态：0-全成功，非0-存在失败
    // 校验状态转移矩阵A(nx×nx)
    status |= check_dimension("State transition matrix (A)", "rows", Adyn.rows(), nx);
    status |= check_dimension("State transition matrix (A)", "columns", Adyn.cols(), nx);
    // 校验输入矩阵B(nx×nu)
    status |= check_dimension("Input matrix (B)", "rows",  Bdyn.rows(), nx);
    status |= check_dimension("Input matrix (B)", "columns",  Bdyn.cols(), nu);
    // 校验仿射偏移向量f(nx×1)
    status |= check_dimension("Affine vector (f)", "rows", fdyn.rows(), nx);
    status |= check_dimension("Affine vector (f)", "columns", fdyn.cols(), 1);
    // 校验状态代价矩阵Q(nx×nx)
    status |= check_dimension("State stage cost (Q)", "rows",  Q.rows(), nx);
    status |= check_dimension("State stage cost (Q)", "columns",  Q.cols(), nx);
    // 校验控制代价矩阵R(nu×nu)
    status |= check_dimension("State input cost (R)", "rows",  R.rows(), nu);
    status |= check_dimension("State input cost (R)", "columns",  R.cols(), nu);
    // 维度校验失败则直接返回错误码，终止初始化
    if (status) {
        return status;
    }
    
    // 8. 初始化工作空间中的核心中间变量：状态/控制序列、代价中间项、对偶变量等，均初始化为零矩阵
    work->x = tinyMatrix::Zero(nx, N);    // 状态序列中间变量
    work->u = tinyMatrix::Zero(nu, N-1);  // 控制序列中间变量
    work->q = tinyMatrix::Zero(nx, N);    // 状态代价中间项
    work->r = tinyMatrix::Zero(nu, N-1);  // 控制代价中间项
    work->p = tinyMatrix::Zero(nx, N);    // 状态相关中间变量
    work->d = tinyMatrix::Zero(nu, N-1);  // 控制相关中间变量

    // 9. 初始化边界约束相关变量：松弛变量（原始变量）、对偶变量，用于ADMM算法处理边界约束
    work->v = tinyMatrix::Zero(nx, N);    // 状态边界约束松弛变量
    work->vnew = tinyMatrix::Zero(nx, N); // 状态边界约束松弛变量新值（ADMM迭代更新）
    work->z = tinyMatrix::Zero(nu, N-1);  // 控制边界约束松弛变量
    work->znew = tinyMatrix::Zero(nu, N-1);// 控制边界约束松弛变量新值
    work->g = tinyMatrix::Zero(nx, N);    // 状态边界约束对偶变量
    work->y = tinyMatrix::Zero(nu, N-1);  // 控制边界约束对偶变量
    
    // 10. 初始化锥约束相关变量：松弛变量、对偶变量，用于ADMM算法处理二阶锥（SOC）约束
    work->vc = tinyMatrix::Zero(nx, N);
    work->vcnew = tinyMatrix::Zero(nx, N);
    work->zc = tinyMatrix::Zero(nu, N-1);
    work->zcnew = tinyMatrix::Zero(nu, N-1);
    work->gc = tinyMatrix::Zero(nx, N);
    work->yc = tinyMatrix::Zero(nu, N-1);

    // 11. 初始化线性约束相关变量：松弛变量、对偶变量，用于ADMM算法处理线性等式/不等式约束
    work->vl = tinyMatrix::Zero(nx, N);
    work->vlnew = tinyMatrix::Zero(nx, N);
    work->zl = tinyMatrix::Zero(nu, N-1);
    work->zlnew = tinyMatrix::Zero(nu, N-1);
    work->gl = tinyMatrix::Zero(nx, N);
    work->yl = tinyMatrix::Zero(nu, N-1);

    // 12. 预处理代价矩阵：融合ADMM惩罚因子rho，转换为对角矩阵（提升后续计算效率）
    // Q = Q + rho*I，R = R + rho*I，取对角元素存储（假设Q/R为对角矩阵，适配嵌入式轻量化计算）
    work->Q = (Q + rho * tinyMatrix::Identity(nx, nx)).diagonal();
    work->R = (R + rho * tinyMatrix::Identity(nu, nu)).diagonal();
    // 13. 保存系统核心矩阵到工作空间：供预计算和实时求解使用
    work->Adyn = Adyn;  // 状态转移矩阵
    work->Bdyn = Bdyn;  // 输入矩阵
    work->fdyn = fdyn;  // 仿射偏移向量

    // 14. 初始化参考轨迹：状态/控制参考轨迹初始化为零矩阵，由外部接口动态设置
    work->Xref = tinyMatrix::Zero(nx, N);
    work->Uref = tinyMatrix::Zero(nu, N-1);

    // 15. 初始化控制相关中间向量：初始化为零向量
    work->Qu = tinyVector::Zero(nu);

    // 16. 初始化求解残差与状态变量：残差初始化为0，迭代次数初始化为0
    work->primal_residual_state = 0;  // 状态原始残差（ADMM收敛判断依据）
    work->primal_residual_input = 0;  // 控制原始残差
    work->dual_residual_state = 0;    // 状态对偶残差
    work->dual_residual_input = 0;    // 控制对偶残差
    work->status = 0;                 // 工作空间状态码
    work->iter = 0;                   // 迭代次数

    // 17. 执行预计算：调用tiny_precompute_and_set_cache，通过Riccati递推预计算核心矩阵并存入缓存
    status = tiny_precompute_and_set_cache(cache, Adyn, Bdyn, fdyn, work->Q.asDiagonal(), work->R.asDiagonal(), nx, nu, rho, verbose);
    if (status) {  // 预计算失败则返回错误码
        return status;
    }

    // 18. 初始化自适应rho的灵敏度矩阵（若开启自适应rho功能）
    if (solver->settings->adaptive_rho) {
        tiny_initialize_sensitivity_matrices(solver);
    }

    return 0;  // 求解器初始化成功，返回0
}

/**
 * @brief 设置MPC状态量/控制量边界约束实现
 * @details 校验边界约束矩阵维度，将约束上下限保存到工作空间，供ADMM求解时使用
 * @param solver 已初始化的MPC求解器实例指针
 * @param x_min 状态量下限(nx×N)：每个时刻每个状态量的最小值
 * @param x_max 状态量上限(nx×N)：每个时刻每个状态量的最大值
 * @param u_min 控制量下限(nu×N-1)：每个时刻每个控制量的最小值
 * @param u_max 控制量上限(nu×N-1)：每个时刻每个控制量的最大值
 * @return int 执行状态：0-成功，1-求解器指针为空，非0-维度校验失败
 */
int tiny_set_bound_constraints(TinySolver* solver,
                    tinyMatrix x_min, tinyMatrix x_max,
                    tinyMatrix u_min, tinyMatrix u_max) {
    // 空指针校验：求解器未初始化则打印错误并返回1
    if (!solver) {
        std::cout << "Error in tiny_set_bound_constraints: solver is nullptr" << std::endl;
        return 1;
    }

    // 校验边界约束矩阵维度：必须与系统维度(nx/nu)和预测步长(N)匹配
    int status = 0;
    status |= check_dimension("Lower state bounds (x_min)", "rows", x_min.rows(), solver->work->nx);
    status |= check_dimension("Lower state bounds (x_min)", "cols", x_min.cols(), solver->work->N);
    status |= check_dimension("Lower state bounds (x_max)", "rows", x_max.rows(), solver->work->nx);
    status |= check_dimension("Lower state bounds (x_max)", "cols", x_max.cols(), solver->work->N);
    status |= check_dimension("Lower input bounds (u_min)", "rows", u_min.rows(), solver->work->nu);
    status |= check_dimension("Lower input bounds (u_min)", "cols", u_min.cols(), solver->work->N-1);
    status |= check_dimension("Lower input bounds (u_max)", "rows", u_max.rows(), solver->work->nu);
    status |= check_dimension("Lower input bounds (u_max)", "cols", u_max.cols(), solver->work->N-1);

    // 将边界约束保存到工作空间，供ADMM求解时调用
    solver->work->x_min = x_min;
    solver->work->x_max = x_max;
    solver->work->u_min = u_min;
    solver->work->u_max = u_max;

    return 0;
}

/**
 * @brief 设置MPC锥约束实现（主要为二阶锥SOC约束）
 * @details 校验锥约束向量维度，保存锥约束参数到工作空间，适配非线性约束场景
 * @param solver 已初始化的MPC求解器实例指针
 * @param Acx 状态量锥约束矩阵索引(整型向量)
 * @param qcx 状态量锥约束参数(整型向量)
 * @param cx 状态量锥约束偏移向量
 * @param Acu 控制量锥约束矩阵索引(整型向量)
 * @param qcu 控制量锥约束参数(整型向量)
 * @param cu 控制量锥约束偏移向量
 * @return int 执行状态：0-成功，1-求解器指针为空，非0-维度校验失败
 */
int tiny_set_cone_constraints(TinySolver* solver,
                              VectorXi Acx, VectorXi qcx, tinyVector cx,
                              VectorXi Acu, VectorXi qcu, tinyVector cu) {
    // 空指针校验
    if (!solver) {
        std::cout << "Error in tiny_set_cone_constraints: solver is nullptr" << std::endl;
        return 1;
    }

    // 获取状态/控制量锥约束的个数（由索引向量的行号决定）
    int num_state_cones = Acx.rows();
    int num_input_cones = Acu.rows();
    // 校验锥约束相关向量维度一致性
    int status = 0;
    status |= check_dimension("Cone state size (qcx)", "rows", qcx.rows(), num_state_cones);
    status |= check_dimension("Cone mu value for state (cx)", "rows", cx.rows(), num_state_cones);
    status |= check_dimension("Cone input size (qcu)", "rows", qcu.rows(), num_input_cones);
    status |= check_dimension("Cone mu value for input (cu)", "rows", cu.rows(), num_input_cones);
    if (status) {  // 维度校验失败返回错误码
        return status;
    }

    // 保存锥约束个数和参数到工作空间
    solver->work->numStateCones = num_state_cones;
    solver->work->numInputCones = num_input_cones;
    solver->work->Acx = Acx;
    solver->work->qcx = qcx;
    solver->work->cx = cx;
    solver->work->Acu = Acu;
    solver->work->qcu = qcu;
    solver->work->cu = cu;

    return 0;
}

/**
 * @brief 设置MPC线性约束实现（等式/不等式约束）
 * @details 校验线性约束矩阵/向量维度，保存线性约束参数到工作空间，处理多变量耦合约束
 * @param solver 已初始化的MPC求解器实例指针
 * @param Alin_x 状态量线性约束矩阵(num_state_linear×nx)
 * @param blin_x 状态量线性约束偏移向量(num_state_linear×1)
 * @param Alin_u 控制量线性约束矩阵(num_input_linear×nu)
 * @param blin_u 控制量线性约束偏移向量(num_input_linear×1)
 * @return int 执行状态：0-成功，1-求解器指针为空，非0-维度校验失败
 */
int tiny_set_linear_constraints(TinySolver* solver,
                               tinyMatrix Alin_x, tinyVector blin_x,
                               tinyMatrix Alin_u, tinyVector blin_u) {
    // 空指针校验
    if (!solver) {
        std::cout << "Error in tiny_set_linear_constraints: solver is nullptr" << std::endl;
        return 1;
    }

    // 获取状态/控制量线性约束的个数（由约束矩阵的行号决定，行号为0表示无该类约束）
    int num_state_linear = Alin_x.rows();
    int num_input_linear = Alin_u.rows();
    int status = 0;
    
    // 状态量线性约束维度校验（仅当存在约束时校验）
    if (num_state_linear > 0) {
        status |= check_dimension("State linear constraint matrix (Alin_x)", "rows", Alin_x.rows(), num_state_linear);
        status |= check_dimension("State linear constraint matrix (Alin_x)", "columns", Alin_x.cols(), solver->work->nx);
        status |= check_dimension("State linear constraint vector (blin_x)", "rows", blin_x.rows(), num_state_linear);
        status |= check_dimension("State linear constraint vector (blin_x)", "columns", blin_x.cols(), 1);
    }
    
    // 控制量线性约束维度校验（仅当存在约束时校验）
    if (num_input_linear > 0) {
        status |= check_dimension("Input linear constraint matrix (Alin_u)", "rows", Alin_u.rows(), num_input_linear);
        status |= check_dimension("Input linear constraint matrix (Alin_u)", "columns", Alin_u.cols(), solver->work->nu);
        status |= check_dimension("Input linear constraint vector (blin_u)", "rows", blin_u.rows(), num_input_linear);
        status |= check_dimension("Input linear constraint vector (blin_u)", "columns", blin_u.cols(), 1);
    }
    
    if (status) {  // 维度校验失败返回错误码
        return status;
    }

    // 保存线性约束个数和参数到工作空间
    solver->work->numStateLinear = num_state_linear;
    solver->work->numInputLinear = num_input_linear;
    solver->work->Alin_x = Alin_x;
    solver->work->blin_x = blin_x;
    solver->work->Alin_u = Alin_u;
    solver->work->blin_u = blin_u;

    return 0;
}

/**
 * @brief MPC核心预计算接口实现
 * @details 核心优化步骤！通过**Riccati递推**预计算MPC求解的关键矩阵（Kinf/Pinf/Quu_inv等），
 *          存入缓存模块避免实时求解时重复计算，大幅提升嵌入式实时求解效率（适配10ms级控制周期）
 * @param cache MPC预计算缓存模块指针
 * @param Adyn/Bdyn/fdyn/Q/R 系统模型与代价函数矩阵（同tiny_setup）
 * @param nx/nu 状态/控制量维度
 * @param rho ADMM算法惩罚因子
 * @param verbose 日志打印开关：1-打印预计算矩阵和收敛信息
 * @return int 执行状态：0-成功，1-缓存指针为空
 */
int tiny_precompute_and_set_cache(TinyCache *cache,
                                  tinyMatrix Adyn, tinyMatrix Bdyn, tinyMatrix fdyn, tinyMatrix Q, tinyMatrix R,
                                  int nx, int nu, tinytype rho, int verbose) {
    // 空指针校验
    if (!cache) {
        std::cout << "Error in tiny_precompute_and_set_cache: cache is nullptr" << std::endl;
        return 1;
    }

    // 1. 代价矩阵融合rho：Q1=Q+rho*I，R1=R+rho*I，引入ADMM惩罚因子，适配ADMM求解框架
    tinyMatrix Q1 = Q + rho * tinyMatrix::Identity(nx, nx);
    tinyMatrix R1 = R + rho * tinyMatrix::Identity(nu, nu);

    // 2. 日志打印：若开启verbose，打印系统核心矩阵和rho值，便于调试
    if (verbose) {
        std::cout << "A = " << Adyn.format(TinyApiFmt) << std::endl;
        std::cout << "B = " << Bdyn.format(TinyApiFmt) << std::endl;
        std::cout << "Q = " << Q1.format(TinyApiFmt) << std::endl;
        std::cout << "R = " << R1.format(TinyApiFmt) << std::endl;
        std::cout << "rho = " << rho << std::endl;
    }

    // 3. 初始化Riccati递推变量：Ktp1/Ptp1为前一迭代值，Kinf/Pinf为收敛后的稳态值
    tinyMatrix Ktp1 = tinyMatrix::Zero(nu, nx);  // 前一迭代的状态反馈增益矩阵
    tinyMatrix Ptp1 = rho * tinyMatrix::Ones(nx, 1).array().matrix().asDiagonal();  // 前一迭代的协方差矩阵
    tinyMatrix Kinf = tinyMatrix::Zero(nu, nx);  // 收敛后的稳态状态反馈增益矩阵（核心输出）
    tinyMatrix Pinf = tinyMatrix::Zero(nx, nx);  // 收敛后的稳态协方差矩阵（核心输出）

    // 4. Riccati递推主循环：最多迭代1000次，直到Kinf收敛（相邻迭代差值小于1e-5）
    for (int i = 0; i < 1000; i++)
    {
        // 计算当前迭代的稳态反馈增益Kinf
        Kinf = (R1 + Bdyn.transpose() * Ptp1 * Bdyn).inverse() * Bdyn.transpose() * Ptp1 * Adyn;
        // 计算当前迭代的稳态协方差Pinf
        Pinf = Q1 + Adyn.transpose() * Ptp1 * (Adyn - Bdyn * Kinf);
        // 收敛判断：Kinf与前一迭代值Ktp1的元素级绝对误差的最大值小于1e-5
        if ((Kinf - Ktp1).cwiseAbs().maxCoeff() < 1e-5)
        {
            if (verbose) {
                std::cout << "Kinf converged after " << i + 1 << " iterations" << std::endl;
            }
            break;  // 收敛则退出递推循环
        }
        // 保存当前迭代值，作为下一迭代的前值
        Ktp1 = Kinf;
        Ptp1 = Pinf;
    }

    // 5. 预计算MPC实时求解所需的核心矩阵/向量：基于Riccati收敛结果，一次计算多次使用
    tinyMatrix Quu_inv = (R1 + Bdyn.transpose() * Pinf * Bdyn).inverse();  // 控制量相关矩阵的逆
    tinyMatrix AmBKt = (Adyn - Bdyn * Kinf).transpose();  // 状态转移矩阵修正项（转置）
    // 仿射项相关预计算：减少实时求解的矩阵乘法操作
    tinyVector APf = AmBKt*Pinf*fdyn;
    tinyVector BPf = Bdyn.transpose()*Pinf*fdyn;

    // 6. 日志打印：若开启verbose，打印所有预计算结果，便于调试和参数调优
    if (verbose) {
        std::cout << "Kinf = " << Kinf.format(TinyApiFmt) << std::endl;
        std::cout << "Pinf = " << Pinf.format(TinyApiFmt) << std::endl;
        std::cout << "Quu_inv = " << Quu_inv.format(TinyApiFmt) << std::endl;
        std::cout << "AmBKt = " << AmBKt.format(TinyApiFmt) << std::endl;
        std::cout << "APf = " << APf.format(TinyApiFmt) << std::endl;
        std::cout << "BPf = " << BPf.format(TinyApiFmt) << std::endl;
        std::cout << "\nPrecomputation finished!\n" << std::endl;
    }

    // 7. 将所有预计算结果保存到缓存模块：供实时求解接口tiny_solve直接调用
    cache->rho = rho;
    cache->Kinf = Kinf;
    cache->Pinf = Pinf;
    cache->Quu_inv = Quu_inv;
    cache->AmBKt = AmBKt;
    cache->C1 = Quu_inv;
    cache->C2 = AmBKt;
    cache->APf = APf;
    cache->BPf = BPf;

    return 0; // 预计算成功，返回0
}

/**
 * @brief MPC核心实时求解接口实现（封装层）
 * @details 作为底层ADMM求解函数solve的封装，对外提供统一的求解入口，
 *          底层solve函数将基于预计算缓存和当前参数，求解最优控制序列
 * @param solver 已初始化的MPC求解器实例指针
 * @return int 执行状态：0-求解收敛成功，非0-求解失败/不收敛（由底层solve函数返回）
 */
int tiny_solve(TinySolver* solver) {
    // 调用底层ADMM求解函数，返回求解状态（核心求解逻辑在solve函数中实现，未在本文件展示）
    return solve(solver);
}

/**
 * @brief 更新MPC求解器配置参数实现
 * @details 动态修改求解器的收敛阈值、迭代次数、约束使能等配置，无需重新初始化求解器，适配实时调优
 * @param settings MPC求解器配置模块指针
 * @param abs_pri_tol 原始残差收敛阈值
 * @param abs_dua_tol 对偶残差收敛阈值
 * @param max_iter 最大迭代次数
 * @param check_termination 收敛检查开关：0-不检查，1-检查
 * @param en_state_bound 状态量边界约束使能：0-关闭，1-开启
 * @param en_input_bound 控制量边界约束使能：0-关闭，1-开启
 * @param en_state_soc 状态量二阶锥约束使能：0-关闭，1-开启
 * @param en_input_soc 控制量二阶锥约束使能：0-关闭，1-开启
 * @param en_state_linear 状态量线性约束使能：0-关闭，1-开启
 * @param en_input_linear 控制量线性约束使能：0-关闭，1-开启
 * @return int 执行状态：0-成功，1-配置指针为空
 */
int tiny_update_settings(TinySettings* settings, tinytype abs_pri_tol, tinytype abs_dua_tol,
                    int max_iter, int check_termination, 
                    int en_state_bound, int en_input_bound,
                    int en_state_soc, int en_input_soc,
                    int en_state_linear, int en_input_linear) {
    // 空指针校验
    if (!settings) {
        std::cout << "Error in tiny_update_settings: settings is nullptr" << std::endl;
        return 1;
    }
    // 赋值新的配置参数到配置模块
    settings->abs_pri_tol = abs_pri_tol;
    settings->abs_dua_tol = abs_dua_tol;
    settings->max_iter = max_iter;
    settings->check_termination = check_termination;
    settings->en_state_bound = en_state_bound;
    settings->en_input_bound = en_input_bound;
    settings->en_state_soc = en_state_soc;
    settings->en_input_soc = en_input_soc;
    settings->en_state_linear = en_state_linear;
    settings->en_input_linear = en_input_linear;
    return 0;
}

/**
 * @brief 设置MPC求解器默认配置参数实现
 * @details 从tiny_api_constants.hpp加载默认常量，初始化配置模块，简化求解器初始化流程
 * @param settings MPC求解器配置模块指针
 * @return int 执行状态：0-成功，1-配置指针为空
 */
int tiny_set_default_settings(TinySettings* settings) {
    // 空指针校验
    if (!settings) {
        std::cout << "Error in tiny_set_default_settings: settings is nullptr" << std::endl;
        return 1;
    }
    // 加载默认收敛阈值、最大迭代次数、收敛检查开关
    settings->abs_pri_tol = TINY_DEFAULT_ABS_PRI_TOL;
    settings->abs_dua_tol = TINY_DEFAULT_ABS_DUA_TOL;
    settings->max_iter = TINY_DEFAULT_MAX_ITER;
    settings->check_termination = TINY_DEFAULT_CHECK_TERMINATION;

    // 初始关闭所有约束：直到外部调用约束设置接口后再开启，避免无约束时的无效计算
    settings->en_state_bound = TINY_DEFAULT_EN_STATE_BOUND;
    settings->en_input_bound = TINY_DEFAULT_EN_INPUT_BOUND;
    settings->en_state_soc = TINY_DEFAULT_EN_STATE_SOC;
    settings->en_input_soc = TINY_DEFAULT_EN_INPUT_SOC;
    settings->en_state_linear = TINY_DEFAULT_EN_STATE_LINEAR;
    settings->en_input_linear = TINY_DEFAULT_EN_INPUT_LINEAR;
    
    // 初始化自适应rho相关配置：默认关闭（仅适配四旋翼等特定系统），设置rho上下限和裁剪开关
    settings->adaptive_rho = 0;  // 0-关闭自适应rho，1-开启
    settings->adaptive_rho_min = 1.0;    // rho最小值
    settings->adaptive_rho_max = 100.0;  // rho最大值
    settings->adaptive_rho_enable_clipping = 1;  // 1-开启rho裁剪（限制在min-max之间）

    return 0;
}

/**
 * @brief 设置MPC求解初始状态x0实现
 * @details 每帧实时更新，传入系统当前实际状态（如云台当前yaw/pitch/角速度），作为MPC求解的起点
 * @param solver 已初始化的MPC求解器实例指针
 * @param x0 初始状态向量(nx×1)
 * @return int 执行状态：0-成功，1-求解器指针为空，非0-维度不匹配
 */
int tiny_set_x0(TinySolver* solver, tinyVector x0) {
    // 空指针校验
    if (!solver) {
        std::cout << "Error in tiny_set_x0: solver is nullptr" << std::endl;
        return 1;
    }
    // 维度校验：x0行号必须等于状态量维度nx
    if (x0.rows() != solver->work->nx) {
        perror("Error in tiny_set_x0: x0 is not the correct length");
    }
    // 将x0赋值给工作空间中状态序列的第一列（k=0时刻的状态）
    solver->work->x.col(0) = x0;
    return 0;
}

/**
 * @brief 设置MPC状态量参考轨迹x_ref实现
 * @details 传入未来N步的状态参考值，求解器将使系统状态尽可能逼近该轨迹（如云台跟踪目标的未来位姿）
 * @param solver 已初始化的MPC求解器实例指针
 * @param x_ref 状态参考轨迹(nx×N)：每列对应未来一步的状态参考值
 * @return int 执行状态：0-成功，1-求解器指针为空，非0-维度校验失败
 */
int tiny_set_x_ref(TinySolver* solver, tinyMatrix x_ref) {
    // 空指针校验
    if (!solver) {
        std::cout << "Error in tiny_set_x_ref: solver is nullptr" << std::endl;
        return 1;
    }
    // 校验x_ref维度：必须为nx×N
    int status = 0;
    status |= check_dimension("State reference trajectory (x_ref)", "rows", x_ref.rows(), solver->work->nx);
    status |= check_dimension("State reference trajectory (x_ref)", "columns", x_ref.cols(), solver->work->N);
    // 将参考轨迹保存到工作空间
    solver->work->Xref = x_ref;
    return 0;
}

/**
 * @brief 设置MPC控制量参考轨迹u_ref实现
 * @details 传入未来N-1步的控制参考值，辅助代价函数优化，可选配置
 * @param solver 已初始化的MPC求解器实例指针
 * @param u_ref 控制参考轨迹(nu×N-1)：每列对应未来一步的控制参考值
 * @return int 执行状态：0-成功，1-求解器指针为空，非0-维度校验失败
 */
int tiny_set_u_ref(TinySolver* solver, tinyMatrix u_ref) {
    // 空指针校验
    if (!solver) {
        std::cout << "Error in tiny_set_u_ref: solver is nullptr" << std::endl;
        return 1;
    }
    // 校验u_ref维度：必须为nu×N-1
    int status = 0;
    status |= check_dimension("Control/input reference trajectory (u_ref)", "rows", u_ref.rows(), solver->work->nu);
    status |= check_dimension("Control/input reference trajectory (u_ref)", "columns",u_ref.cols(), solver->work->N-1);
    // 将控制参考轨迹保存到工作空间
    solver->work->Uref = u_ref;
    return 0;
}

/**
 * @brief 初始化自适应rho的灵敏度矩阵实现
 * @details 为ADMM算法的自适应惩罚因子rho提供梯度信息，预加载离线计算的灵敏度矩阵（适配特定系统），
 *          当rho动态调整时，可快速更新预计算缓存，提升求解鲁棒性
 * @param solver 已初始化的MPC求解器实例指针
 */
void tiny_initialize_sensitivity_matrices(TinySolver *solver) {
    // 获取系统维度参数
    int nu = solver->work->nu;
    int nx = solver->work->nx;
    // 1. 初始化灵敏度矩阵为零矩阵：预留内存空间
    solver->cache->dKinf_drho = tinyMatrix::Zero(nu, nx);  // Kinf对rho的偏导数
    solver->cache->dPinf_drho = tinyMatrix::Zero(nx, nx);  // Pinf对rho的偏导数
    solver->cache->dC1_drho = tinyMatrix::Zero(nu, nu);    // C1对rho的偏导数
    solver->cache->dC2_drho = tinyMatrix::Zero(nx, nx);    // C2对rho的偏导数

    // 2. 预加载离线计算的灵敏度矩阵常量（适配特定系统，如四旋翼/云台，由仿真或实验得到）
    // Kinf对rho的偏导数矩阵(4×12)
    const float dKinf_drho[4][12] = {
        {  0.0001,  -0.0001,  -0.0025,   0.0003,   0.0007,   0.0050,   0.0001,  -0.0001,  -0.0008,   0.0000,   0.0001,   0.0008},
        { -0.0001,  -0.0000,  -0.0025,  -0.0001,  -0.0006,  -0.0050,  -0.0001,   0.0000,  -0.0008,  -0.0000,  -0.0001,  -0.0008},
        {  0.0000,   0.0000,  -0.0025,   0.0001,   0.0004,   0.0050,   0.0000,   0.0000,  -0.0008,   0.0000,   0.0000,   0.0008},
        { -0.0000,   0.0001,  -0.0025,  -0.0003,  -0.0004,  -0.0050,  -0.0000,   0.0001,  -0.0008,  -0.0000,  -0.0000,  -0.0008}
    };
    // Pinf对rho的偏导数矩阵(12×12)
    const float dPinf_drho[12][12] = {
        {  0.0494,  -0.0045,  -0.0000,   0.0110,   0.1300,  -0.0283,   0.0280,  -0.0026,  -0.0000,   0.0004,   0.0070,  -0.0094},
        { -0.0045,   0.0491,   0.0000,  -0.1320,  -0.0111,   0.0114,  -0.0026,   0.0279,   0.0000,  -0.0076,  -0.0004,   0.0038},
        { -0.0000,   0.0000,   2.4450,   0.0000,  -0.0000,  -0.0000,  -0.0000,   0.0000,   1.2593,   0.0000,   0.0000,   0.0000},
        {  0.0110,  -0.1320,   0.0000,   0.3913,   0.0592,   0.3108,   0.0080,  -0.0776,   0.0000,   0.0254,   0.0068,   0.0750},
        {  0.1300,  -0.0111,  -0.0000,   0.0592,   0.4420,   0.7771,   0.0797,  -0.0081,  -0.0000,   0.0068,   0.0350,   0.1875},
        { -0.0283,   0.0114,  -0.0000,   0.3108,   0.7771,  10.0441,   0.0272,  -0.0109,   0.0000,   0.0655,   0.1639,   2.6362},
        {  0.0280,  -0.0026,  -0.0000,   0.0080,   0.0797,   0.0272,   0.0163,  -0.0016,  -0.0000,   0.0005,   0.0047,   0.0032},
        { -0.0026,   0.0279,   0.0000,  -0.0776,  -0.0081,  -0.0109,  -0.0016,   0.0161,   0.0000,  -0.0046,  -0.0005,  -0.0013},
        { -0.0000,   0.0000,   1.2593,   0.0000,  -0.0000,   0.0000,  -0.0000,   0.0000,   0.9232,   0.0000,   0.0000,   0.0000},
        {  0.0004,  -0.0076,   0.0000,   0.0254,   0.0068,   0.0655,   0.0005,  -0.0046,   0.0000,   0.0022,   0.0017,   0.0244},
        {  0.0070,  -0.0004,   0.0000,   0.0068,   0.0350,   0.1639,   0.0047,  -0.0005,   0.0000,   0.0017,   0.0054,   0.0610},
        { -0.0094,   0.0038,   0.0000,   0.0750,   0.1875,   2.6362,   0.0032,  -0.0013,   0.0000,   0.0244,   0.0610,   0.9869}
    };
    // C1对rho的偏导数矩阵(4×4)
    const float dC1_drho[4][4] = {
        { -0.0000,   0.0000,  -0.0000,   0.0000},
        {  0.0000,  -0.0000,   0.0000,  -0.0000},
        { -0.0000,   0.0000,  -0.0000,   0.0000},
        {  0.0000,  -0.0000,   0.0000,  -0.0000}
    };
    // C2对rho的偏导数矩阵(12×12)
    const float dC2_drho[12][12] = {
        {  0.0000,  -0.0000,   0.0000,   0.0000,   0.0000,  -0.0000,   0.0000,  -0.0000,   0.0000,   0.0000,   0.0000,  -0.0000},
        { -0.0000,   0.0000,   0.0000,  -0.0000,  -0.0000,   0.0000,  -0.0000,   0.0000,   0.0000,  -0.0000,  -0.0000,   0.0000},
        { -0.0000,   0.0000,   0.0001,   0.0000,  -0.0000,  -0.0000,  -0.0000,   0.0000,   0.0000,   0.0000,  -0.0000,  -0.0000},
        {  0.0000,  -0.0000,  -0.0000,   0.0001,   0.0000,  -0.0000,   0.0000,  -0.0000,  -0.0000,   0.0000,   0.0000,  -0.0000},
        {  0.0000,  -0.0000,  -0.0000,   0.0000,   0.0001,  -0.0000,   0.0000,  -0.0000,  -0.0000,   0.0000,   0.0000,  -0.0000},
        { -0.0000,   0.0000,  -0.0000,  -0.0000,   0.0000,   0.0001,  -0.0000,   0.0000,  -0.0000,   0.0000,   0.0000,   0.0000},
        {  0.0000,  -0.0000,   0.0000,   0.0000,   0.0000,  -0.0000,   0.0000,  -0.0000,   0.0000,   0.0000,   0.0000,  -0.0000},
        { -0.0000,   0.0000,   0.0000,  -0.0000,  -0.0000,   0.0000,  -0.0000,   0.0000,   0.0000,  -0.0000,  -0.0000,   0.0000},
        { -0.0000,   0.0000,   0.0021,   0.0000,  -0.0000,  -0.0000,  -0.0000,   0.0000,   0.0006,   0.0000,  -0.0000,  -0.0000},
        {  0.0002,  -0.0027,  -0.0000,   0.0068,   0.0005,  -0.0005,   0.0001,  -0.0015,  -0.0000,   0.0004,   0.0000,  -0.0001},
        {  0.0027,  -0.0002,   0.0000,   0.0005,   0.0066,  -0.0011,   0.0015,  -0.0001,   0.0000,   0.0000,   0.0004,  -0.0002},
        { -0.0001,   0.0001,   0.0000,  -0.0000,   0.0000,   0.0041,  -0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0006}
    };

    // 3. 将静态数组映射为Eigen矩阵，并转换为tinytype类型，保存到缓存模块
    solver->cache->dKinf_drho = Map<const Matrix<float, 4, 12>>(dKinf_drho[0]).cast<tinytype>();
    solver->cache->dPinf_drho = Map<const Matrix<float, 12, 12>>(dPinf_drho[0]).cast<tinytype>();
    solver->cache->dC1_drho = Map<const Matrix<float, 4, 4>>(dC1_drho[0]).cast<tinytype>();
    solver->cache->dC2_drho = Map<const Matrix<float, 12, 12>>(dC2_drho[0]).cast<tinytype>();
}

// 结束C/C++混合调用兼容宏
#ifdef __cplusplus
}
#endif
