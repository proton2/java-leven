kernel void sum(global const float* a, global const float* b, global float* result, int const size)
{
    const int itemId = get_global_id(0);
    if(itemId < size) {
        result[itemId] = a[itemId] + b[itemId];
    }
}