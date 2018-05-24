class StRingBuffer
{
  public:
    StRingBuffer(int len);
    String addChar(char val);
    String getString();
    void clear();
  private:
    int _length;
    int _pos;
    String _data;
};

StRingBuffer::StRingBuffer(int len)
{
  _length = len;
  clear();
}

void StRingBuffer::clear()
{
  _data = "";
  for (_pos = 0; _pos < _length; _pos++)
    _data += " ";
  _pos = 0;
}

String StRingBuffer::addChar(char val)
{
  if (!isPrintable(val))
    val = ' ';
  _data[_pos] = val;
  _pos = (++_pos)%_length;
  return getString();
}

String StRingBuffer::getString()
{
  if (_pos > 0)
    return _data.substring(_pos) + _data.substring(0, _pos);
  else
    return _data;
}

