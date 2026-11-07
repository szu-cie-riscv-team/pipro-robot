/** Suspend UI result polling while hidden; no timer or background worker. */
export function whenVisible(): Promise<void> {
  if (typeof document === 'undefined' || !document.hidden) return Promise.resolve();
  return new Promise(resolve => {
    const visible = () => {
      if (document.hidden) return;
      document.removeEventListener('visibilitychange', visible);
      resolve();
    };
    document.addEventListener('visibilitychange', visible);
  });
}
