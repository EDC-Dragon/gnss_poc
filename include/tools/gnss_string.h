#pragma once

#include <stddef.h>

typedef struct
{
    char **data;
    size_t count;
} StringSplitResult;

StringSplitResult *string_split(const char *str, char delimiter);
void string_split_free(StringSplitResult *res);