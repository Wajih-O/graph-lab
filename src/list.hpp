#include <ostream>
namespace mylib
{
template <class T>
class List
{
  T *head;
  T *tail;
  T *cursor;
  List<T> *next;
  List<T> *prev;

public:
  // static inline const char *name = "JUST for testing";
  List() : head(nullptr), cursor(head), tail(head), next(nullptr), prev(nullptr) {}
  explicit List(T element) : head(new T(element)), cursor(head), tail(head), next(nullptr), prev(nullptr) {}

  T *get_head() const { return head; }
  T *get_cursor() const { return cursor; }
  List<T> *get_next() const { return next; }
  T *get_tail() const {return tail;}
  
  friend std::ostream &operator<<(std::ostream &out, const List<T> &l)
  {
    out << *l.get_head();
    List<T> *nxt = l.get_next();
    while (nxt != nullptr)
    {
      out << ", " << *nxt->get_head();
      nxt = nxt->get_next();
    }
    out << ", eol.";
    return out;
  }

  List<T> *prepend(T to_prepend)
  {
    List<T> *temp = new List(to_prepend);
    temp->next = this;
    this->prev = temp;
    return temp;
  }
};
}
