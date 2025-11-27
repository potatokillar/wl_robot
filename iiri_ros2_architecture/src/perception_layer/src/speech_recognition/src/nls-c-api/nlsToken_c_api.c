/*
 * @Author: 唐文浩
 * @Date: 2024-11-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2024-11-21
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "nlsToken_c_api.h"

NlsTokenHandle nls_token_create_2() { return nls_token_create(); }
void nls_token_destroy_2(NlsTokenHandle handle) { nls_token_destroy(handle); }
void nls_token_setAccessKeyId_2(NlsTokenHandle handle, const char *accessKeyId) { nls_token_setAccessKeyId(handle, accessKeyId); }
void nls_token_setKeySecret_2(NlsTokenHandle handle, const char *KeySecret) { nls_token_setKeySecret(handle, KeySecret); };
void nls_token_setDomain_2(NlsTokenHandle handle, const char *Domain) { nls_token_setKeySecret(handle, Domain); };
void nls_token_setServerVersion_2(NlsTokenHandle handle, const char *serverVersion) { nls_token_setKeySecret(handle, serverVersion); };