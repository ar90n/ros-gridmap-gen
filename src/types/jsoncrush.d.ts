declare module 'jsoncrush' {
  const JSONCrush: {
    crush(input: string): string;
    uncrush(input: string): string;
  };
  export default JSONCrush;
}