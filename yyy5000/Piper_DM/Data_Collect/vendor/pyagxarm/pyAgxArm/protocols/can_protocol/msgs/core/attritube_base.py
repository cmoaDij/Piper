from typing import Optional, Tuple

class AttributeBase:
    """
    协议 / 消息 / 属性 基类
    - 支持字段顺序控制
    - 支持嵌套属性自动缩进
    - 统一 str / repr / dict / eq 行为
    """

    _fields_: Optional[Tuple[str, ...]] = None
    _derived_fields_: Tuple[str, ...] = ()
    _indent_: str = "  "   # 每一层增加的缩进（2 空格）

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        cls.type_ = cls.__name__

    # ---------- internal ----------
    def _iter_items(self):
        """
        按协议顺序迭代 (key, value)
        """
        if self._fields_:
            for k in self._fields_:
                yield k, getattr(self, k)
        else:
            for k, v in self.__dict__.items():
                if not k.startswith("_"):
                    yield k, v
        # ⭐ 派生字段（只读属性 / 计算属性）
        for k in self._derived_fields_:
            yield k, getattr(self, k)

    def _format_value(self, value, indent: str):
        """
        根据值类型格式化（支持嵌套 Attribute）
        """
        if isinstance(value, AttributeBase):
            # 子属性：换行 + 增加一层缩进
            return "\n" + value._to_str(indent + self._indent_)
        return str(value)

    def _to_str_with_name(self, indent: str):
        cls_name = self.__class__.__name__
        body = self._to_str(indent)
        return f"{indent}{cls_name}:\n{body}"
        
    def _format_value(self, value, indent: str):
        if isinstance(value, AttributeBase):
            # 子 Attribute：打印其完整结构（包含类名）
            # return "\n" + value._to_str_with_name(indent + self._indent_)
            return value._to_str_with_name(indent + self._indent_)
        return str(value)

    def _to_str(self, indent: str):
        """
        生成当前层级的字符串（不包含类名）
        """
        lines = []
        for k, v in self._iter_items():
            formatted_value = self._format_value(v, indent)
            lines.append(
                f"{indent}{self._indent_}{k}: {formatted_value}"
            )
        return "\n".join(lines)

    # ---------- public ----------
    def __str__(self):
        cls_name = self.__class__.__name__
        body = self._to_str(indent="")
        return f"{cls_name}:\n{body}"

    def __repr__(self):
        cls_name = self.__class__.__name__
        args = ", ".join(f"{k}={v!r}" for k, v in self._iter_items())
        return f"{cls_name}({args})"

    def to_dict(self) -> dict:
        """
        转为 dict（递归）
        """
        result = {}
        for k, v in self._iter_items():
            if isinstance(v, AttributeBase):
                result[k] = v.to_dict()
            else:
                result[k] = v
        return result

    def __eq__(self, other):
        """
        值相等判断（同类 + 字段值完全一致）
        """
        if self is other:
            return True
        if other.__class__ is not self.__class__:
            return False
        return self.to_dict() == other.to_dict()