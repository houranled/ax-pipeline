#include "Thread.h"

int THREAD_CREATE(THREAD *thread, THREAD_FUNC func(void *), void *param)
{
#ifdef WIN32
	*thread = (HANDLE)_beginthreadex(NULL, 0, func, param, 0, NULL);
	return *thread == NULL ? -1 : 0;
#else
	return pthread_create(thread, NULL, func, param);
#endif
}

int THREAD_JOIN(THREAD *thread)
{
#ifdef WIN32
	return WaitForSingleObject(*thread, INFINITE) == WAIT_FAILED ? -1 : 0;
#else
	// 注意：pthread_join 会向第二个参数指向的位置写入一个 void*（64 位平台 8 字节）。
	// 原实现传 &int 会在 aarch64/x86_64 上越界写坏栈 canary，触发 __stack_chk_fail。
	// 返回值这里未使用，直接传 NULL。
	return pthread_join(*thread, NULL);
#endif
}

void THREAD_DESTROY(THREAD *thread)
{
#ifdef WIN32
	if (*thread)
		CloseHandle(*thread);
#endif
	*thread = NULL;
}
