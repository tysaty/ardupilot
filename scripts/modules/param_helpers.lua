-- parameter helpers - generic
-- local table
local M = {}

-- binding parameteres
-- currently not used
function M.bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s", name))
    return p
end

-- add params
-- used in kanagroo.lua
function M.bind_add_param(table_key, prefix, name, idx, default_value)
    assert(param:add_param(table_key, idx, name, default_value), string.format("could not add param %s", prefix .. name))
    return M.bind_param(prefix .. name)
end

return M