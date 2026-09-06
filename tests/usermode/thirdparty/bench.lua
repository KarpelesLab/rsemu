-- A Lua program for the guest to run: a sieve, coroutines, string patterns, a
-- table sort and a float.
--
-- Written to exercise the *runtime* rather than the syscall table, because
-- that half of a real program is where a core's arithmetic and a garbage
-- collector's pointer chasing get tested and a hello world tests neither.
local function sieve(n)
  local is = {}
  for i = 2, n do is[i] = true end
  for i = 2, math.floor(math.sqrt(n)) do
    if is[i] then
      for j = i * i, n, i do is[j] = false end
    end
  end
  local c = 0
  for i = 2, n do if is[i] then c = c + 1 end end
  return c
end

local co = coroutine.wrap(function()
  for i = 1, 5 do coroutine.yield(i * i) end
end)
local squares = {}
for _ = 1, 5 do squares[#squares + 1] = co() end

local words = {}
for w in ("the quick brown fox jumps over the lazy dog"):gmatch("%a+") do
  words[#words + 1] = w:upper()
end
table.sort(words)

print("primes below 20000: " .. sieve(20000))
print("squares: " .. table.concat(squares, ","))
print("words: " .. table.concat(words, " "))
print(string.format("float: %.6f", 2 ^ 0.5 * math.pi))
print("version: " .. _VERSION)
