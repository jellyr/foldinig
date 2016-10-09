//
//  simplifyPath.h
//  RDP
//
//  Created by Yongjiao Yu on 14-6-13.
//  Copyright (c) 2014”N ___RCPD___. All rights reserved.
//


#ifndef __simplifyPath__H__
#define __simplifyPath__H__


//"PointH" struct stand for each GPS coordinates(x,y). Methods related are used only for simplification of calculation in implementation.

typedef struct PointH{
    double x;
    double y;
    
    double operator*(PointH rhs)const{
        return (this->x * rhs.y - rhs.x * this->y);
    }
    
    PointH operator-(PointH rhs)const{
        PointH p;
        p.x=this->x-rhs.x;
        p.y=this->y-rhs.y;
        return p;
    }
    
    double Norm()const{
        return sqrt(this->x * this->x + this->y * this->y);
    }
    
}PointH;

class simplifyPath{
//"findMaximumDistance" used as part of implementation for RDP algorithm.
public:
    const std::pair<int, double> findMaximumDistance(const std::vector<PointH>& Points)const;
    
//"simplifyWithRDP" returns the simplified path with a PointH vector. The function takes in the paths to be simplified and a customerized thresholds for the simplication.
public:
    std::vector<PointH> simplifyWithRDP(std::vector<PointH>& Points, double epsilon)const;
};

#endif 
