define flash
file build/ch.elf
load
end

define reconnect
target extended-remote localhost:4242
file build/ch.elf
end

reconnect
