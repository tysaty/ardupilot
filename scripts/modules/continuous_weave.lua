-- Updated weaving module

local math_helpers = require("math_helpers")
local clamp = math_helpers.clamp

-- Smooth activation curve
-- q = 0 -> no weave
-- q = 1 -> full weave
function smoothstep(q)
    return 3*q*q - 2*q*q*q
end

-- Desired weave amplitude based on distance to kangaroo
function desired_weave_amplitude(distance_to_kangaroo, A_cap, d_start, d_full)
    -- d_start distance where weave begins
    -- d_full distance where weave reaches full amplitude
    --  d_start > d_full

    local q = (d_start - distance_to_kangaroo) / (d_start - d_full)
    q = clamp(q, 0, 1)
    return A_cap *q
    
end

-- Constrain amplitude using minimum turn radius
function constrained_weave_amplitude(distance_to_kangaroo, lambda, R_min, A_cap, d_start, d_full, eta)

    local A_desired = desired_weave_amplitude(distance_to_kangaroo, A_cap, d_start,d_full)

    -- Curvature-based amplitude limit for y = A sin(2*pi*s/lambda)
    local A_turn_limit = eta * (lambda * lambda) / (4 * math.pi * math.pi * R_min)

    return math.min(A_desired, A_turn_limit)
end

-- Generate straight-line + weave reference point
function straight_weave(plane_x, plane_y, kangaroo_x, kangaroo_y,u, s, lambda, R_min, A_cap, d_start, d_full, eta, phase)

    -- Vector from UAV to kangaroo
    local dx = kangaroo_x - plane_x
    local dy = kangaroo_y - plane_y

    local distance_to_kangaroo = math.sqrt(dx*dx + dy*dy)

    -- Avoid division by zero
    if distance_to_kangaroo < 1e-6 then
        return kangaroo_x, kangaroo_y
    end

    -- Unit direction toward kangaroo
    local tx = dx / distance_to_kangaroo
    local ty = dy / distance_to_kangaroo

    -- Unit normal to the straight-line path
    -- This gives lateral left/right weave
    local nx = -ty
    local ny = tx

    -- Straight-line reference point
    -- u = 0 gives current UAV position
    -- u = 1 gives kangaroo position
    local line_x = plane_x + u * dx
    local line_y = plane_y + u * dy

    -- Distance-based, turn-radius-constrained amplitude
    local A = constrained_weave_amplitude(distance_to_kangaroo, lambda, R_min, A_cap, d_start, d_full, eta)

    -- Sine weave offset
    local weave = A * math.sin((2 * math.pi * s / lambda) + phase)

    -- Final reference point accounted for with the unit looking forward
    local ref_x = line_x + nx * weave
    local ref_y = line_y + ny * weave

    return ref_x, ref_y
end


return {
    straight_weave = straight_weave
}
