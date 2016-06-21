#pragma once

static bool& draw_axis =
    CVarUtils::CreateCVar<>("masseuse.DrawAxis", false, "");
static double& stiffness_multiplier =
    CVarUtils::CreateCVar<>("masseuse.StiffnessFactor", 1.0, "");
static double& cov_det_treshold =
    CVarUtils::CreateCVar<>("masseuse.CovarianceDeterminantThreshold", 1e-35, "");
static int& num_iterations =
    CVarUtils::CreateCVar<>("masseuse.NumIterations",10000, "");


