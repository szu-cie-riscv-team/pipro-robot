import { id } from './core';
import { whenVisible } from './visibility';
export type WarehouseRequest = (operation: string, args?: any, requestId?: string) => Promise<any>;
/** Only outstanding warehouse operation; reconnect never replays the mutation. */
export class WarehouseOperation {
  pending?: { requestId: string; operation: string; operationId?: string };
  constructor(private request: WarehouseRequest) {}
  async start(operation: string, args: any): Promise<any> {
    if (this.pending) throw new Error('上次结果未知，请先查询原操作结果');
    this.pending = { requestId: id(), operation };
    let response;
    try { response = await this.request(operation, args, this.pending.requestId); }
    catch (error) {
      if ((error as any)?.rejected) { this.pending = undefined; throw error; }
      // The request may have reached the board. Query the original identity exactly once, never sync again.
      return this.resume();
    }
    this.pending.operationId = response.operation_id;
    return this.complete(response.result);
  }
  async resume(): Promise<any> {
    if (!this.pending) throw new Error('没有待查询的仓库操作');
    await whenVisible();
    const response = await this.request('get_operation', { request_id: this.pending.requestId });
    return this.complete(response.result);
  }
  private async complete(result: any): Promise<any> {
    const deadline = Date.now() + 26000;
    while (result.status === 'running') {
      await new Promise(resolve => setTimeout(resolve, 400));
      await whenVisible();
      result = (await this.request('get_operation', { request_id: this.pending!.requestId })).result;
      if (result.status === 'running' && Date.now() >= deadline) throw new Error('仓库操作结果未知，请查询原操作');
    }
    if (!['succeeded', 'failed'].includes(result.status)) throw new Error('仓库操作结果未知');
    this.pending = undefined;
    if (result.status === 'failed') throw new Error(result.error || '仓库操作失败');
    return result.result;
  }
}
