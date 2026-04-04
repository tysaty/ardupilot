-- Math helpers 
-- local table
local M = {}

-- clamping values
function M.clamp(value, min_value, max_value)
    if value < min_value then
        return min_value
    end
    if value > max_value then
        return max_value
    end
    return value
end

-- setting floor values
function M.ordered_range(v1, v2, floor_value)
    local a = math.max(v1, floor_value)
    local b = math.max(v2, floor_value)
    if a > b then
        a, b = b, a
    end
    return a, b
end

-- random values
function M.random_between(min_value, max_value)
    if max_value <= min_value then
        return min_value
    end
    return min_value + (max_value - min_value) * math.random()
end

-- floors for angles
function M.wrap_360(angle)
    local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

return M