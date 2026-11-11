// Runs TSA/tests/test_stick_alt.lua with fengari (a Lua 5.3 VM in JavaScript),
// for machines with Node.js but no Lua.
//
//   npm install fengari@0.1.4      (anywhere; set NODE_PATH if not in this folder)
//   node TSA/tests/run.js          (from the repo root)

const path = require('path');
const { lua, lauxlib, lualib, to_luastring, to_jsstring } = require('fengari');

const root = path.resolve(__dirname, '..', '..');
const fwd = (p) => p.replace(/\\/g, '/');
const script = fwd(path.join(root, 'TSA', 'scripts', 'stick_alt.lua'));
const test = fwd(path.join(root, 'TSA', 'tests', 'test_stick_alt.lua'));

const L = lauxlib.luaL_newstate();
lualib.luaL_openlibs(L);
lua.lua_pushstring(L, to_luastring(script));
lua.lua_setglobal(L, to_luastring('SCRIPT_PATH'));

if (lauxlib.luaL_dofile(L, to_luastring(test)) !== lua.LUA_OK) {
  console.error(to_jsstring(lua.lua_tostring(L, -1)));
  process.exit(1);
}
