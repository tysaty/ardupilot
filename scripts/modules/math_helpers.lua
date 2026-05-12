-- =========================================================
--  Helper table
-- =========================================================

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

local PI = math.pi

-- pi wrapping
function M.wrap_pi(i)
    while i > PI do
        i = i - 2.0 * PI 
    end
    while i < -PI do 
        i = i + 2.0 * PI 
    end
    return i
end


-- Euclidean distance
function M.dist2d(x1, y1, x2, y2)
    local dx = x2 - x1
    local dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)
end

-- Degree to E7
function M.deg_to_e7(deg)
    if deg == nil then
        return nil
    end
    if deg >= 0 then
        return math.floor(deg * 1.0e7 + 0.5)
    end
    return math.ceil(deg * 1.0e7 - 0.5)
end

-- ---------------------------------------------------------
-- Matrix helpers
-- ---------------------------------------------------------

-- multiplication
function M.mat_mul(A, B)
    local res = {}
    for i=1, #A do
        res[i] = {}
        for j=1,#B[1] do
            res[i][j] = 0
            for k=1, #B do
                res[i][j] = res[i][j] + A[i][k] * B[k][j]
            end
        end
    end
    return res
end

-- addition
function M.mat_add(A, B)
    local res = {}
    for i=1, #A do
        res[i] = {}
        for j=1,#A[1] do
            res[i][j] = A[i][j] + B[i][j]
        end
    end
    return res
end

-- subtraction
function M.mat_sub(A, B)
    local res = {}
    for i = 1, #A do
        res[i] = {}
        for j = 1, #A[1] do
            res[i][j] = A[i][j] - B[i][j]
        end
    end
    return res
end

-- transpose
function M.transpose(A)
    local res = {}
    for i=1,#A[1] do
        res[i] = {}
        for j=1,#A do
            res[i][j] = A[j][i]
        end
    end
    return res
end

-- zeroes
function M.zeros(r, c)
    local res = {}
    for i = 1, r do
        res[i] = {}
        for j = 1, c do
            res[i][j] = 0
        end
    end
    return res
end

-- identity matrix
function M.eye(n)
    local res = M.zeros(n, n)
    for i = 1, n do
        res[i][i] = 1
    end
    return res
end

-- vector to column
function M.vec_to_col(v)
    local res = {}
    for i = 1, #v do
        res[i] = {v[i]}
    end
    return res
end

-- column to vector
function M.col_to_vec(A)
    local res = {}
    for i = 1, #A do
        res[i] = A[i][1]
    end
    return res
end

-- 2x2 matrix inversion
function M.invert_22(A)
    local a = A[1][1]
    local b = A[1][2]
    local c = A[2][1]
    local d = A[2][2]
    -- determinant
    local det = a * d - b * c
    -- handle nil case or negative
    if math.abs(det) < 1e-9 then
        return nil
    end
    -- else return the matrix inverted
    return {
        { d / det, -b / det},
        {-c / det,  a / det}
    }
end

return M