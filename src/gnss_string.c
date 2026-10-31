#include "gnss_string.h"

#include <stdlib.h>
#include <string.h>

StringSplitResult *string_split(const char *str, char delimiter)
{
    if (str == NULL)
        return NULL;

    // Ignorer les retours à la ligne en fin de chaîne.
    size_t length = strlen(str);

    while (length > 0 &&
           (str[length - 1] == '\n' || str[length - 1] == '\r'))
        length--;

    // Compter les champs.
    size_t n = 1;

    for (size_t i = 0; i < length; i++)
        if (str[i] == delimiter)
            n++;

    char **result = malloc(n * sizeof(*result));

    if (result == NULL)
        return NULL;

    size_t start = 0;
    size_t field = 0;

    for (size_t i = 0; i <= length; i++)
    {
        if (i != length && str[i] != delimiter)
            continue;

        size_t len = i - start;

        result[field] = malloc(len + 1);

        if (result[field] == NULL)
        {
            for (size_t j = 0; j < field; j++)
                free(result[j]);

            free(result);
            return NULL;
        }

        memcpy(result[field], str + start, len);
        result[field][len] = '\0';

        field++;
        start = i + 1;
    }

    StringSplitResult *res = malloc(sizeof(*res));

    if (res == NULL)
    {
        for (size_t i = 0; i < n; i++)
            free(result[i]);

        free(result);
        return NULL;
    }

    res->data = result;
    res->count = n;

    return res;
}
void string_split_free(StringSplitResult *res)
{
    if (res == NULL || res->data == NULL)
        return;

    for (size_t i = 0; i < res->count; i++)
        free(res->data[i]);

    free(res->data);
    free(res);
}