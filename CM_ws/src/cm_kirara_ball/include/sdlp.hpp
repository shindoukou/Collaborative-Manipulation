 








 















#ifndef SDLP_HPP
#define SDLP_HPP

#include <Eigen/Eigen>
#include <cmath>
#include <random>

namespace sdlp
{
    constexpr double eps = 1.0e-12;

    enum
    {
         
        MINIMUM = 0,
         
        INFEASIBLE,
         
        UNBOUNDED,
         
        AMBIGUOUS,
    };

    inline double dot2(const double a[2],
                       const double b[2])
    {
        return a[0] * b[0] + a[1] * b[1];
    }

    inline double cross2(const double a[2],
                         const double b[2])
    {
        return a[0] * b[1] - a[1] * b[0];
    }

    inline bool unit2(const double a[2],
                      double b[2])
    {
        const double mag = std::sqrt(a[0] * a[0] +
                                     a[1] * a[1]);
        if (mag < 2.0 * eps)
        {
            return true;
        }
        b[0] = a[0] / mag;
        b[1] = a[1] / mag;
        return false;
    }

     
    template <int d>
    inline bool unit(double *a)
    {
        double mag = 0.0;
        for (int i = 0; i <= d; i++)
        {
            mag += a[i] * a[i];
        }
        if (mag < (d + 1) * eps * eps)
        {
            return true;
        }
        mag = 1.0 / std::sqrt(mag);
        for (int i = 0; i <= d; i++)
        {
            a[i] *= mag;
        }
        return false;
    }

     
    template <int d>
    inline int lp_no_con(const double *n_vec,
                         const double *d_vec,
                         double *opt)
    {
        double n_dot_d = 0.0;
        double d_dot_d = 0.0;
        for (int i = 0; i <= d; i++)
        {
            n_dot_d += n_vec[i] * d_vec[i];
            d_dot_d += d_vec[i] * d_vec[i];
        }
        if (d_dot_d < eps * eps)
        {
            n_dot_d = 0.0;
            d_dot_d = 1.0;
        }
        for (int i = 0; i <= d; i++)
        {
            opt[i] = -n_vec[i] +
                     d_vec[i] * n_dot_d / d_dot_d;
        }
         
        if (unit<d>(opt))
        {
            opt[d] = 1.0;
            return AMBIGUOUS;
        }
        else
        {
            return MINIMUM;
        }
    }

     
    inline int move_to_front(const int i,
                             int *next,
                             int *prev)
    {
        if (i == 0 || i == next[0])
        {
            return i;
        }
        const int previ = prev[i];
         
        next[prev[i]] = next[i];
        prev[next[i]] = prev[i];
         
        next[i] = next[0];
        prev[i] = 0;
        prev[next[i]] = i;
        next[0] = i;
        return previ;
    }

    inline void lp_min_lin_rat(const bool degen,
                               const double cw_vec[2],
                               const double ccw_vec[2],
                               const double n_vec[2],
                               const double d_vec[2],
                               double opt[2])
    {
         
        const double d_cw = dot2(cw_vec, d_vec);
        const double d_ccw = dot2(ccw_vec, d_vec);
        const double n_cw = dot2(cw_vec, n_vec);
        const double n_ccw = dot2(ccw_vec, n_vec);
        if (degen)
        {
             
            if (n_cw / d_cw < n_ccw / d_ccw)
            {
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
            else
            {
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
             
        }
        else if (std::fabs(d_cw) > 2.0 * eps &&
                 std::fabs(d_ccw) > 2.0 * eps)
        {
             
            if (d_cw * d_ccw > 0.0)
            {
                 
                if (n_cw / d_cw < n_ccw / d_ccw)
                {
                    opt[0] = cw_vec[0];
                    opt[1] = cw_vec[1];
                }
                else
                {
                    opt[0] = ccw_vec[0];
                    opt[1] = ccw_vec[1];
                }
            }
            else
            {
                 
                if (d_cw > 0.0)
                {
                    opt[0] = -d_vec[1];
                    opt[1] = d_vec[0];
                }
                else
                {
                    opt[0] = d_vec[1];
                    opt[1] = -d_vec[0];
                }
            }
        }
        else if (std::fabs(d_cw) > 2.0 * eps)
        {
             
            if (n_ccw * d_cw > 0.0)
            {
                 
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
            else
            {
                 
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
        }
        else if (std::fabs(d_ccw) > 2.0 * eps)
        {
             
            if (n_cw * d_ccw > 2.0 * eps)
            {
                 
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
            else
            {
                 
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
        }
        else
        {
             
            if (cross2(d_vec, n_vec) > 0.0)
            {
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
            }
            else
            {
                opt[0] = ccw_vec[0];
                opt[1] = ccw_vec[1];
            }
        }
    }

    inline int wedge(const double (*halves)[2],
                     const int m,
                     int *next,
                     int *prev,
                     double cw_vec[2],
                     double ccw_vec[2],
                     bool *degen)
    {
        int i;
        double d_cw, d_ccw;
        bool offensive;

        *degen = false;
        for (i = 0; i != m; i = next[i])
        {
            if (!unit2(halves[i], ccw_vec))
            {
                 
                cw_vec[0] = ccw_vec[1];
                cw_vec[1] = -ccw_vec[0];
                 
                ccw_vec[0] = -cw_vec[0];
                ccw_vec[1] = -cw_vec[1];
                break;
            }
        }
        if (i == m)
        {
            return UNBOUNDED;
        }
        i = 0;
        while (i != m)
        {
            offensive = false;
            d_cw = dot2(cw_vec, halves[i]);
            d_ccw = dot2(ccw_vec, halves[i]);
            if (d_ccw >= 2.0 * eps)
            {
                if (d_cw <= -2.0 * eps)
                {
                    cw_vec[0] = halves[i][1];
                    cw_vec[1] = -halves[i][0];
                    unit2(cw_vec, cw_vec);
                    offensive = true;
                }
            }
            else if (d_cw >= 2.0 * eps)
            {
                if (d_ccw <= -2.0 * eps)
                {
                    ccw_vec[0] = -halves[i][1];
                    ccw_vec[1] = halves[i][0];
                    unit2(ccw_vec, ccw_vec);
                    offensive = true;
                }
            }
            else if (d_ccw <= -2.0 * eps &&
                     d_cw <= -2.0 * eps)
            {
                return INFEASIBLE;
            }
            else if (d_cw <= -2.0 * eps ||
                     d_ccw <= -2.0 * eps ||
                     cross2(cw_vec, halves[i]) < 0.0)
            {
                 
                if (d_cw <= -2.0 * eps)
                {
                    unit2(ccw_vec, cw_vec);
                }
                else if (d_ccw <= -2.0 * eps)
                {
                    unit2(cw_vec, ccw_vec);
                }
                *degen = true;
                offensive = true;
            }
             
            if (offensive)
            {
                i = move_to_front(i, next, prev);
            }
            i = next[i];
            if (*degen)
            {
                break;
            }
        }
        if (*degen)
        {
            while (i != m)
            {
                d_cw = dot2(cw_vec, halves[i]);
                d_ccw = dot2(ccw_vec, halves[i]);
                if (d_cw < -2.0 * eps)
                {
                    if (d_ccw < -2.0 * eps)
                    {
                        return INFEASIBLE;
                    }
                    else
                    {
                        cw_vec[0] = ccw_vec[0];
                        cw_vec[1] = ccw_vec[1];
                    }
                }
                else if (d_ccw < -2.0 * eps)
                {
                    ccw_vec[0] = cw_vec[0];
                    ccw_vec[1] = cw_vec[1];
                }
                i = next[i];
            }
        }
        return MINIMUM;
    }

     
    inline int lp_base_case(const double (*halves)[2],  
                            const int m,                
                            const double n_vec[2],      
                            const double d_vec[2],      
                            double opt[2],              
                            int *next,                  
                            int *prev)
    {
        double cw_vec[2], ccw_vec[2];
        bool degen;
        int status;

         
        status = wedge(halves, m, next, prev, cw_vec, ccw_vec, &degen);

        if (status == INFEASIBLE)
        {
            return status;
        }
         
        if (status == UNBOUNDED)
        {
            return lp_no_con<1>(n_vec, d_vec, opt);
        }

        if (std::fabs(cross2(n_vec, d_vec)) < 2.0 * eps * eps)
        {
            if (dot2(n_vec, n_vec) < 2.0 * eps * eps ||
                dot2(d_vec, d_vec) > 2.0 * eps * eps)
            {
                 
                opt[0] = cw_vec[0];
                opt[1] = cw_vec[1];
                status = AMBIGUOUS;
            }
            else
            {
                 
                if (!degen &&
                    cross2(cw_vec, n_vec) <= 0.0 &&
                    cross2(n_vec, ccw_vec) <= 0.0)
                {
                     
                    opt[0] = -n_vec[0];
                    opt[1] = -n_vec[1];
                }
                else if (dot2(n_vec, cw_vec) > dot2(n_vec, ccw_vec))
                {
                     
                    opt[0] = ccw_vec[0];
                    opt[1] = ccw_vec[1];
                }
                else
                {
                     
                    opt[0] = cw_vec[0];
                    opt[1] = cw_vec[1];
                }
                status = MINIMUM;
            }
        }
        else
        {
             
            lp_min_lin_rat(degen, cw_vec, ccw_vec, n_vec, d_vec, opt);
            status = MINIMUM;
        }
        return status;
    }

     
    template <int d>
    inline void findimax(const double *pln,
                         int *imax)
    {
        *imax = 0;
        double rmax = std::fabs(pln[0]);
        for (int i = 1; i <= d; i++)
        {
            const double ab = std::fabs(pln[i]);
            if (ab > rmax)
            {
                *imax = i;
                rmax = ab;
            }
        }
    }

    template <int d>
    inline void vector_up(const double *equation,
                          const int ivar,
                          const double *low_vector,
                          double *vector)
    {
        vector[ivar] = 0.0;
        for (int i = 0; i <= d; i++)
        {
            if (i != ivar)
            {
                const int j = i < ivar ? i : i - 1;
                vector[i] = low_vector[j];
                vector[ivar] -= equation[i] * low_vector[j];
            }
        }
        vector[ivar] /= equation[ivar];
    }

    template <int d>
    inline void vector_down(const double *elim_eqn,
                            const int ivar,
                            const double *old_vec,
                            double *new_vec)
    {
        double ve = 0.0;
        double ee = 0.0;
        for (int i = 0; i <= d; i++)
        {
            ve += old_vec[i] * elim_eqn[i];
            ee += elim_eqn[i] * elim_eqn[i];
        }
        const double fac = ve / ee;
        for (int i = 0; i <= d; i++)
        {
            if (i != ivar)
            {
                new_vec[i < ivar ? i : i - 1] =
                    old_vec[i] - elim_eqn[i] * fac;
            }
        }
    }

    template <int d>
    inline void plane_down(const double *elim_eqn,
                           const int ivar,
                           const double *old_plane,
                           double *new_plane)
    {
        const double crit = old_plane[ivar] / elim_eqn[ivar];
        for (int i = 0; i <= d; i++)
        {
            if (i != ivar)
            {
                new_plane[i < ivar ? i : i - 1] =
                    old_plane[i] - elim_eqn[i] * crit;
            }
        }
    }

    template <int d>
    inline int linfracprog(const double *halves,  
                           const int max_size,    
                           const int m,           
                           const double *n_vec,   
                           const double *d_vec,   
                           double *opt,           
                           double *work,          
                           int *next,             
                           int *prev)             
     



























    {
        int status, imax;
        double *new_opt, *new_n_vec, *new_d_vec, *new_halves, *new_work;
        const double *plane_i;

        double val = 0.0;
        for (int j = 0; j <= d; j++)
        {
            val += d_vec[j] * d_vec[j];
        }
        const bool d_vec_zero = (val < (d + 1) * eps * eps);

         
        status = lp_no_con<d>(n_vec, d_vec, opt);
        if (m <= 0)
        {
            return status;
        }

         
        new_opt = work;
        new_n_vec = new_opt + d;
        new_d_vec = new_n_vec + d;
        new_halves = new_d_vec + d;
        new_work = new_halves + max_size * d;
        for (int i = 0; i != m; i = next[i])
        {
             
            plane_i = halves + i * (d + 1);
             
            val = 0.0;
            for (int j = 0; j <= d; j++)
            {
                val += opt[j] * plane_i[j];
            }
            if (val < -(d + 1) * eps)
            {
                 
                findimax<d>(plane_i, &imax);
                 
                if (i != 0)
                {
                    const double fac = 1.0 / plane_i[imax];
                    for (int j = 0; j != i; j = next[j])
                    {
                        const double *old_plane = halves + j * (d + 1);
                        const double crit = old_plane[imax] * fac;
                        double *new_plane = new_halves + j * d;
                        for (int k = 0; k <= d; k++)
                        {
                            const int l = k < imax ? k : k - 1;
                            new_plane[l] = k != imax ? old_plane[k] - plane_i[k] * crit : new_plane[l];
                        }
                    }
                }
                 
                if (d_vec_zero)
                {
                    vector_down<d>(plane_i, imax, n_vec, new_n_vec);
                    for (int j = 0; j < d; j++)
                    {
                        new_d_vec[j] = 0.0;
                    }
                }
                else
                {
                    plane_down<d>(plane_i, imax, n_vec, new_n_vec);
                    plane_down<d>(plane_i, imax, d_vec, new_d_vec);
                }
                 
                status = linfracprog<d - 1>(new_halves, max_size, i, new_n_vec,
                                            new_d_vec, new_opt, new_work, next, prev);
                 
                if (status != INFEASIBLE)
                {
                    vector_up<d>(plane_i, imax, new_opt, opt);

                     
                    double mag = 0.0;
                    for (int j = 0; j <= d; j++)
                    {
                        mag += opt[j] * opt[j];
                    }
                    mag = 1.0 / sqrt(mag);
                    for (int j = 0; j <= d; j++)
                    {
                        opt[j] *= mag;
                    }
                }
                else
                {
                    return status;
                }
                 
                i = move_to_front(i, next, prev);
            }
        }
        return status;
    }

    template <>
    inline int linfracprog<1>(const double *halves,
                              const int max_size,
                              const int m,
                              const double *n_vec,
                              const double *d_vec,
                              double *opt,
                              double *work,
                              int *next,
                              int *prev)
    {
        if (m > 0)
        {
            return lp_base_case((const double(*)[2])halves, m,
                                n_vec, d_vec, opt, next, prev);
        }
        else
        {
            return lp_no_con<1>(n_vec, d_vec, opt);
        }
    }

    inline void rand_permutation(const int n,
                                 int *p)
    {
        typedef std::uniform_int_distribution<int> rand_int;
        typedef rand_int::param_type rand_range;
        static std::mt19937_64 gen;
        static rand_int rdi(0, 1);
        int j, k;
        for (int i = 0; i < n; i++)
        {
            p[i] = i;
        }
        for (int i = 0; i < n; i++)
        {
            rdi.param(rand_range(0, n - i - 1));
            j = rdi(gen) + i;
            k = p[j];
            p[j] = p[i];
            p[i] = k;
        }
    }

    template <int d>
    inline double linprog(const Eigen::Matrix<double, d, 1> &c,
                          const Eigen::Matrix<double, -1, d> &A,
                          const Eigen::Matrix<double, -1, 1> &b,
                          Eigen::Matrix<double, d, 1> &x)
     



    {
        int m = b.size() + 1;
        x.setZero();
        if (m <= 1)
        {
            return c.cwiseAbs().maxCoeff() > 0.0 ? -INFINITY : 0.0;
        }

        Eigen::VectorXi perm(m - 1);
        Eigen::VectorXi next(m);
         
        Eigen::VectorXi prev(m + 1);
        Eigen::Matrix<double, d + 1, 1> n_vec;
        Eigen::Matrix<double, d + 1, 1> d_vec;
        Eigen::Matrix<double, d + 1, 1> opt;
        Eigen::Matrix<double, d + 1, -1, Eigen::ColMajor> halves(d + 1, m);
        Eigen::VectorXd work((m + 3) * (d + 2) * (d - 1) / 2);

        halves.col(0).setZero();
        halves(d, 0) = 1.0;
        halves.topRightCorner(d, m - 1) = -A.transpose();
        halves.bottomRightCorner(1, m - 1) = b.transpose();
         
        halves.colwise().normalize();
        n_vec.head(d) = c;
        n_vec(d) = 0.0;
        d_vec.setZero();
        d_vec(d) = 1.0;

         
        rand_permutation(m - 1, perm.data());
         
        prev(0) = 0;
         
        next(0) = perm(0) + 1;
        prev(perm(0) + 1) = 0;
         
        for (int i = 0; i < m - 2; i++)
        {
            next(perm(i) + 1) = perm(i + 1) + 1;
            prev(perm(i + 1) + 1) = perm(i) + 1;
        }
         
        next(perm(m - 2) + 1) = m;

        int status = sdlp::linfracprog<d>(halves.data(), m, m,
                                          n_vec.data(), d_vec.data(),
                                          opt.data(), work.data(),
                                          next.data(), prev.data());

         
        double minimum = INFINITY;
        if (status != sdlp::INFEASIBLE)
        {
            if (opt(d) != 0.0 && status != sdlp::UNBOUNDED)
            {
                x = opt.head(d) / opt(d);
                minimum = c.dot(x);
            }

            if (opt(d) == 0.0 || status == sdlp::UNBOUNDED)
            {
                x = opt.head(d);
                minimum = -INFINITY;
            }
        }

        return minimum;
    }

}  

#endif
