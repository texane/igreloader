static inline void fubar(int fu, int bar, int baz)
{
  asm("nop");
}

int main(int ac)
{
  fubar(ac, 42, 3);
  return 0;
}
